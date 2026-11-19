# esc_can.lua

Índice da pasta: [README](README.md).

Driver Lua de ESC DJI C620 / motor M3508 para o ArduPilot Rover, sem firmware modificado. Versão marcada no código: `v2611o`.

Copia para `APM/scripts` no cartão SD junto com `rover-controlallocation.lua` e `modules/autotune.lua`. O driver fala CAN com os ESCs, fecha um PI de velocidade em cada roda e publica o estado para o mixer e para a estação de solo.

## O que ele faz

A cada 5 ms, com `ESC_CAN = 1`:

1. Zera o PWM das saídas MAIN 1 a 4. Qualquer pulso, inclusive o trim de 1500 µs, tira o C620 do modo CAN.
2. Esvazia o buffer CAN e fica só com o frame mais novo de cada ESC.
3. Converte a rotação do rotor em rpm de roda.
4. Lê o comando do lado esquerdo e do direito, em −1 a +1.
5. Transforma esse comando em rpm de referência e calcula a corrente de cada roda.
6. Envia a corrente no protocolo do C620.
7. Se um ESC que já tinha sido visto ficar 100 ms sem feedback, corta a corrente de todos.

Com `ESC_CAN = 0` o script não escreve no barramento. O C620 segue o pulso PWM normal do Rover.

## Como deve atuar

### Ligar e trocar de modo

`ESC_CAN` só muda com o veículo desarmado. Armado, o valor pedido é ignorado e a GCS recebe `ESC: disarm to switch ESC_CAN` no máximo a cada 5 s.

Ao entrar em CAN, nesta ordem:

- as MAIN 1 a 4 passam a largura 0, com override de 3 s;
- o buffer de recepção é esvaziado, para um frame da época do PWM não virar feedback;
- a mensagem é `ESC: CAN ON (reactive), PWM zero`.

Ao voltar para PWM a mensagem é `ESC: CAN OFF (PWM)` e o barramento fica em silêncio. O estado dos ESCs, os integradores, o comando e o autotune são zerados na troca.

O driver começa em PWM até conseguir ler `ESC_CAN`. Começar já em CAN zeraria as MAIN 1 a 4 antes de saber o modo.

### De onde vem o comando

O comando de cada lado é um número de −1 a +1. +1 pede `ESC_RPM` de roda para a frente.

| `SCR_USER5` | Fonte |
| --- | --- |
| 2 | `rover-controlallocation.lua` escreve `ESC_CMD_L`, `ESC_CMD_R` e incrementa `ESC_CMD_SEQ`. Este driver só aceita o par quando `ESC_CMD_SEQ` muda. O primeiro valor depois do boot ou de uma troca de modo não conta: ainda pode ser resto do boot anterior. Sem um número de sequência novo por 500 ms, os dois lados vão a 0. |
| outro | Saídas do Rover. `ThrottleLeft` (função 73) e `ThrottleRight` (função 74) vêm em −1000 a +1000 e são divididas por 1000. Se as duas estão em 0, `Throttle` (função 70), em −100 a +100, é dividido por 100 e vale para os dois lados. |

Desarmado, os dois comandos são forçados a 0 depois dessa leitura. O stick não move o motor.

### Velocidade ou corrente

`ESC_MODE = 0` é o modo normal. Os dois ESCs de um lado recebem a mesma rpm de referência (`comando do lado × ESC_RPM`). Cada um fecha o próprio PI no próprio encoder. Uma roda no ar não zera o erro da roda que ainda está no chão.

`ESC_MODE = 1` não fecha velocidade. A corrente de cada roda é `comando do lado × ESC_CUR`, limitada a ±`ESC_CUR`.

### Parado, com o stick no centro

No modo velocidade, com o comando do lado quase 0 (menos de 1%), a referência quase 0 e a roda abaixo de 5 rpm, a corrente vai a 0 e o integrador é limpo. Segurar 0 rpm através da folga do redutor e do ruído do encoder faz o motor zunir. Isso vale armado.

A exceção é o bit 1 de `ESC_OPT`. Com ele ligado, o PI continua fechado em 0 rpm mesmo parado, e também com o veículo desarmado, desde que o modo seja velocidade, a máscara de lados seja válida, não haja estop, a safety esteja liberada e todo ESC esperado esteja saudável. Sem esse bit, desarmado significa corrente 0: as rodas ficam livres.

### Ré e lado da roda

`ESC_REV` é uma máscara. O bit 0 é o ESC 1. O bit ligado multiplica por −1 a rpm medida e a corrente enviada, de modo que corrente positiva continue sendo "para a frente" no referencial do veículo.

`ESC_LMASK` diz quem é o lado esquerdo. O bit 0 é o ESC 1. Bit ligado = esquerdo (lado 1); bit desligado = direito (lado 2). O valor 9 é ESC 1 e ESC 4 à esquerda, ESC 2 e ESC 3 à direita. O padrão gravado na tabela é 6: ESC 2 e ESC 3 à esquerda.

A máscara 0 e a máscara 0xFF são inválidas: todo mundo no mesmo lado. Em CAN isso recusa o armar, manda erro a cada 5 s e não habilita a corrente.

`ESC_GEAR` é a redução rotor/roda. No M3508 com redutor P19 o valor é 19,2032. A rpm de roda é `sentido × rpm do rotor / ESC_GEAR`.

### Rampa, pré-filtro e frenagem

`ESC_RPM_AUX` é a chave geral dos auxiliares da malha de RPM, com padrão 1. Com 1, cada recurso segue seu parâmetro individual. Com 0, a referência passa sem a rampa de `ESC_ACC` e sem o pré-filtro de `ESC_PREF`; a janela de corrente fica em ±`ESC_CUR` sem a restrição adicional de `ESC_BRK`; o boost de `ESC_STALL` e a dessaturação do bit 2 de `ESC_OPT` ficam desativados. P/I/D/FF, anti-windup, limites de corrente/integrador, tratamento de neutro e proteções de comando/feedback continuam ativos. Os valores individuais gravados não são alterados. O bypass também é usado na verificação em malha fechada do autotune de roda.

A mudança é lida em até aproximadamente 50 ms. Ao voltar para 1, o filtro parte da referência acompanhada durante o bypass. Desligar o boost também pode aumentar o tempo para vencer atrito na partida; desativar suavização não garante menos overshoot. Para comparar o efeito no seguimento de reta, mantenha os ganhos e os outros parâmetros iguais entre os ensaios. Depois, os parâmetros individuais permitem isolar cada recurso.

`ESC_RPM_AUX` ocupa a nova posição 47, sem deslocar as posições existentes. Após instalar o `esc_can.lua` atualizado, reinicie a controladora para recriar a tabela com o tamanho novo; só reiniciar o script pode deixar a tabela antiga alocada. Com o script carregado e `SCR_ENABLE = 1`, use **Refresh Params** no Mission Planner para localizar `ESC_RPM_AUX` no **Full Parameter List**. Grave o valor pela estação de solo para persistir após reboot.

`ESC_ACC` limita a aceleração da referência, em rpm de roda por segundo, nos dois sentidos. 0 deixa a referência igual ao alvo no mesmo ciclo. O uso previsto no stick e na missão é 0, sem rampa artificial.

`ESC_PREF` liga um filtro `1 / (1 + Ti·s)` na referência, com `Ti = Kp / Ki` daquela roda. A constante de tempo é `Ti × ESC_PREF`. O PI do ótimo simétrico, sem esse filtro, ultrapassa um degrau de velocidade em cerca de 40%. Com os ganhos sintonizados, 0,5 derruba o overshoot para cerca de 4% e acrescenta 0,09 s; 1,0 tira o overshoot e acrescenta 0,28 s. O filtro só atua quando o módulo da referência está crescendo. Reduzir e parar passam direto. Troca de sinal zera o estado do filtro antes de seguir o novo lado.

`ESC_BRK = 0` deixa a corrente em ±`ESC_CUR` o tempo todo. Corrente contra o giro é permitida.

`ESC_BRK = 1` (o padrão) restringe a corrente contrária só enquanto a roda já gira a mais de 5 rpm e a referência pede para manter ou acelerar nesse sentido. Nesse caso o lado contrário fica em `ESC_BRKCUR` por cento de `ESC_CUR`. Desacelerar, parar ou inverter abre de novo ±`ESC_CUR`, como um pulso PWM invertido.

### Saturação e patinação

O bit 2 de `ESC_OPT` liga a dessaturação. Se um lado saturou no ciclo anterior e o outro não, a referência do lado livre é multiplicada pela fração que o lado saturado conseguiu cumprir. A razão esquerda/direita do comando permanece, e o raio da curva não abre. Se as rodas daquele lado divergiram, a fração usa a roda mais atrasada. A roda que está no ar não conta como sucesso.

Duas rodas do mesmo lado divergiram quando a diferença de avanço entre a mais rápida e a mais lenta passa de `ESC_SLIP`, ou, com `ESC_SLIP = 0`, de 10% de `ESC_RPM` com mínimo de 40 rpm, e o comando daquele lado passa de 1%.

`ESC_SLIP` maior que 0 também gera aviso. Se uma roda fica mais de 100 ms afastada da média do próprio lado por mais do que `ESC_SLIP` rpm, `ESC: slip on ESC n` sai no máximo a cada 5 s e o bit de patinação sobe em `ESC_STAT`. O aviso não muda a corrente. Cada PI continua no próprio encoder.

`ESC_STAT` é só leitura entre scripts. Não salvar. Bits:

| Bit | Valor | Significado |
| --- | --- | --- |
| 0 | 1 | CAN ativo e o driver Lua existe |
| 1 | 2 | Algum lado saturou nos últimos 100 ms |
| 2 | 4 | Autotune de roda em andamento |
| 3 | 8 | Autotune terminou e gravou os ganhos |
| 4 | 16 | Autotune falhou; os ganhos anteriores ficaram |
| 5 | 32 | Patinação ativa |

O mixer lê o bit 0 para saber que este driver está em CAN.

### O que corta a corrente

A corrente só sai se tudo isto for verdade:

- CAN já confirmado;
- safety liberada;
- sem emergency stop;
- `ESC_LMASK` com ESC nos dois lados;
- todo ESC já detectado, e todo servo com função 70, 73 ou 74, com feedback há menos de 100 ms;
- veículo armado, ou desarmado com o bit 1 de `ESC_OPT` no modo velocidade.

Fora isso os integradores e as referências zeram e o frame CAN leva 0 A. Perder o feedback de um ESC que já tinha aparecido, com o veículo armado, também gera `ESC: ESC n lost` em severidade crítica.

Um erro de execução não deixa o script morrer. Se ele parasse, o override de PWM expiraria em 3 s e o Rover voltaria a pulsar as entradas PWM do C620. No primeiro erro o driver trava até o reboot: 0 A no barramento, MAIN 1 a 4 em largura 0, armar recusado, e a cada 5 s a GCS recebe `ESC: fault, motors stopped; reboot` mais os primeiros 40 caracteres do erro.

## Parâmetros que o Rover precisa ter

| Parâmetro | Valor |
| --- | --- |
| `CAN_Px_DRIVER` | 1 ou 2, o driver de scripting |
| `CAN_Px_BITRATE` | 1000000 |
| `CAN_Dx_PROTOCOL` | 10 (Scripting) |
| `SCR_VM_I_COUNT` | 100000. Abaixo de 50000 o script avisa e pede reboot. O laço de 5 ms estoura a cota padrão. |
| `SCR_USER5` | 2 para o mixer Lua mandar `ESC_CMD_L/R`. Outro valor faz este driver ler as saídas de servo. |
| `SCR_USER1` | 0 no uso normal. 4 ou 5 disparam o autotune de roda. 6 não faz mais nada. |

Se `CAN_P1_DRIVER` e `CAN_P2_DRIVER` apontam para o mesmo número, só o CAN1 é usado. O driver de scripting fica com a primeira porta mapeada para ele.

A tabela de parâmetros usa a chave 74 e o prefixo `ESC_`. Se a 74 já estiver ocupada por outro prefixo, tenta a 72. As chaves são criadas na carga do script, antes do laço, para uma falha posterior não esconder a tabela. É preciso reboot para a estação de solo passar a listar parâmetros novos.

`ESC_CMD_L`, `ESC_CMD_R`, `ESC_CMD_SEQ` e `ESC_STAT` são ponte entre scripts, só em RAM. Não salvar.

## Parâmetros ESC_

Índice na tabela, nome, padrão e função. Ganho 0 numa roda significa "usar o ganho compartilhado".

| # | Nome | Padrão | Função |
| --- | --- | --- | --- |
| 1 | `ESC_CAN` | 0 | 0 = PWM, script calado no barramento. 1 = CAN. Só troca desarmado. |
| 2 | `ESC_MODE` | 0 | 0 = PI de velocidade por roda. 1 = corrente proporcional ao comando. |
| 3 | `ESC_RPM` | 400 | Rpm de roda no comando 1,0. Também limita os patamares do autotune. |
| 4 | `ESC_GEAR` | 19,2032 | Redução rotor/roda. Mínimo efetivo 1. |
| 5 | `ESC_CUR` | 10 | Limite de corrente, A. O script ainda corta em 20 A, fundo de escala do C620. |
| 6 | `ESC_P` | 0,03 | Kp compartilhado. Erro em rpm de roda, saída em A. |
| 7 | `ESC_I` | 0,3 | Ki compartilhado, A por rpm por segundo. |
| 8 | `ESC_D` | 0 | Kd compartilhado. Não há D por roda. |
| 9 | `ESC_FF` | 0 | Feed-forward compartilhado, A por rpm de referência. |
| 10 | `ESC_IMAX` | 10 | Módulo máximo do integrador, A. Também limitado a `ESC_CUR`. |
| 11 | `ESC_REV` | 0 | Máscara de inversão. Bit 0 = ESC 1. |
| 12 | `ESC_OPT` | 1 | Bit 0 grava o log `DJIC`. Bit 1 segura 0 rpm desarmado. Bit 2 dessatura o outro lado. O padrão liga só o log. |
| 13 | `ESC_BRK` | 1 | Ver a seção de frenagem. |
| 14 | `ESC_ACC` | 0 | Limite de aceleração da referência, rpm/s. 0 = sem rampa. |
| 15 | `ESC_BRKCUR` | 30 | Corrente contrária permitida com `ESC_BRK = 1`, em % de `ESC_CUR`, enquanto segura ou acelera. |
| 16 | `ESC_LOC_P` | 0 | Sem uso. A correção antiga de lado foi trocada pelo PI por roda. |
| 17 | `ESC_LOC_MAX` | 15 | Sem uso. Mantido para a tabela não deslocar. |
| 18 | `ESC_SLIP` | 60 | Diferença de rpm roda-contra-média do lado que conta como patinação. 0 desliga o aviso. A divergência usada na dessaturação continua, com 10% de `ESC_RPM`. |
| 19 | `ESC_LMASK` | 6 | Máscara do lado esquerdo. 6 = ESC 2 e ESC 3. |
| 20 | `ESC_WT_RELAY` | 100 | Corrente do degrau do autotune, % de `ESC_CUR`, de 1 a 100. O degrau nunca fica abaixo de 0,5 A. |
| 21 | `ESC_WT_RPM` | 120 | Patamar alto do autotune, rpm de roda. Abaixo de 60 o script usa 120, para um 30 antigo salvo na EEPROM não assumir. O patamar baixo é a metade. |
| 22 | `ESC_CMD_L` | 0 | Comando esquerdo, −1 a +1. Escrito pelo mixer. Não salvar. |
| 23 | `ESC_CMD_R` | 0 | Comando direito. Não salvar. |
| 24 | `ESC_CMD_SEQ` | 0 | Contador do mixer. Mudança aplica L e R na hora. Não salvar. |
| 25 | `ESC_STAT` | 0 | Bits de estado. Este script zera na partida. Não salvar. |
| 26 | `ESC_LOC_I` | 0,5 | Sem uso. |
| 27–30 | `ESC_FR1` … `ESC_FR4` | 0 | Sem uso. Mantidos para a tabela não deslocar. |
| 31–42 | `ESC_P1` `ESC_I1` `ESC_FF1` … `P4` `I4` `FF4` | 0 | PI e feed-forward da roda. 0 usa `ESC_P`, `ESC_I`, `ESC_FF`. |
| 43 | `ESC_GCS_HZ` | 4 | `NAMED_VALUE_FLOAT` para a GCS, Hz. 0 desliga. Teto 20. |
| 44 | `ESC_PREF` | 0,5 | Pré-filtro da referência, fração de Ti. 0 desliga. Teto 2. |
| 45 | `ESC_WT_BW` | 8 | Banda desejada do autotune, rad/s. No cálculo fica entre 1 e 50 e ainda é cortada por `1 / (2,65 T)`. |
| 46 | `ESC_STALL` | 30 | Boost de destravamento, A/s. 0 desliga. |
| 47 | `ESC_RPM_AUX` | 1 | 0 contorna os cinco auxiliares descritos acima; 1 usa suas configurações individuais. |

Os ganhos não são lidos a 200 Hz. O cache (`M.gains`) refresca a cada 50 ms. `ESC_REV`, `ESC_LMASK` e a função de cada `SERVO1`…`SERVO8` refrescam a cada 1 s.

## Lei de controle

Para cada roda detectada, no modo velocidade, com as saídas habilitadas e sem o autotune no comando:

1. A referência sobe ou desce no máximo `ESC_ACC × dt` em direção ao alvo do lado. Com `ESC_ACC = 0` ela copia o alvo.
2. Se o pré-filtro está ligado, essa referência passa por ele. A constante é `Ti × ESC_PREF`, e `Ti = Kp / Ki` já com a troca do ganho 0 pelo ganho compartilhado.
3. O erro é `referência − rpm de roda`.
4. A corrente não saturada é `FF × referência + Kp × erro + integrador + Kd × (erro − erro anterior) / dt`.
5. O integrador só avança quando essa conta não está empurrando para fora da janela de corrente no mesmo sentido do erro. Ele fica dentro de ±`ESC_IMAX`.
6. A saída é essa mesma conta, sem reutilizar o integrador novo, limitada à janela de `current_window`.

Kp, Ki e FF saem de `ESC_P1`…`ESC_FF4` quando o valor da roda é maior que 0. Kd é sempre `ESC_D`.

A roda é marcada saturada quando a conta passa de ±`ESC_CUR` com o erro no mesmo sentido. É essa marca, atrasada um ciclo, que a dessaturação lê. A janela de frenagem pode limitar antes, sem acender a marca.

Unidade: erro em rpm de roda, corrente em ampere.

## CAN do C620

O dispositivo Lua é `CAN:get_device(25)`. O filtro `0x7F0 / 0x200` deixa no buffer só os identificadores `0x200` a `0x20F`, para outro tráfego não expulsar o feedback.

Quatro C620 enviam cerca de 4000 frames por segundo para um buffer de 25 frames. O buffer, cheio, descarta o frame novo. O script lê até 40 frames por ciclo, guarda só o mais recente de cada ESC e decodifica esse. O laço trabalha com dado deste ciclo.

Feedback aceito: identificador padrão `0x201` a `0x208` (`0x200 + número do ESC`), DLC 8, sem extended, sem RTR, sem error frame. O byte layout do C620:

| Bytes | Campo |
| --- | --- |
| 0–1 | Ângulo do rotor, 0 a 8191 contagens por volta |
| 2–3 | Rpm do rotor, int16 big-endian |
| 4–5 | Corrente, int16. 16384 = 20 A |
| 6 | Temperatura, °C |

Como frames são pulados, o ângulo pode dar várias voltas entre duas amostras. A posição integrada do log `DJIC` usa a rpm informada, não a diferença de ângulo.

Comando, também int16 big-endian, fundo de escala ±16384 = ±20 A:

| Quem | Identificador | Conteúdo |
| --- | --- | --- |
| ESC 1 a 4 | `0x200` | Quatro correntes, ESC 1 no primeiro par de bytes |
| ESC 5 a 8 | `0x1FF` | Quatro correntes, ESC 5 no primeiro par |

O grupo só é enviado se algum ESC daquele grupo já foi detectado. A corrente no fio é `sentido × corrente da roda × (16384 / 20)`.

Na partida de cada ESC a GCS recebe `ESC: found ESC n (left|right)`.

Desarmado, com algum servo de motor ainda sem ESC, a cada 5 s sai uma linha de diagnóstico: contagem de frames nos IDs 1 a 4, soma dos IDs 5 a 8, frames de outro identificador e o maior intervalo entre laços, em ms. Serve para separar ESC mudo, ID errado e laço lento.

## Autotune de roda

`SCR_USER1 = 4` sintoniza todas as rodas detectadas, no chão, armado, modo HOLD, sticks de direção e acelerador a menos de 100 µs do trim. `SCR_USER1 = 5` faz a mesma sequência uma roda por vez, com as outras em 0 A: as rodas têm de estar no ar. Rodar o 5 antes do 4. O 4 regrava com a massa do veículo. `SCR_USER1 = 6` está aposentado.

`ESC_MODE` tem de ser 0. Os sticks não podem sair do centro. Sair de HOLD, desarmar ou mudar o seletor aborta. Perder feedback falha. Em qualquer falha os ganhos anteriores permanecem.

Há duas definições de `M.tune_start_direction` no arquivo. A de baixo substitui a de cima. A de cima não cria as listas do degrau e não é a que roda.

Estados, nesta ordem, primeiro para a frente e depois para trás. No modo 5, entre uma roda e a seguinte há 2 s com tudo em 0 A.

| Estado | O que faz |
| --- | --- |
| `ff` | Segura o patamar baixo e depois o alto com o PI atual. Mede 1 s depois de 1 s de acomodação. Se a rpm média ficar a mais de 30% do alvo, falha. A inclinação de corrente entre os dois patamares vira `ESC_FFn` daquela roda. Inclinação negativa cai para corrente/rpm do patamar alto. |
| `step` | No patamar alto, soma e depois subtrai um degrau de corrente (`ESC_WT_RELAY` % de `ESC_CUR`). Cada roda tem a própria guarda: ao se afastar mais de metade do patamar alto em relação à rpm em que o degrau começou, volta à corrente de espera. O degrau dura no máximo 0,25 s. A reta `rpm = a t + b` usa só amostras que já andaram pelo menos 4 rpm no sentido do degrau. `Kv` é a inclinação por ampere. `T` é o instante em que a reta cruza a rpm inicial. Atraso entre 0 e −50 ms vira 0. Atraso acima de 300 ms ou Kv não positivo são descartados. |
| `recover` | 0,8 s de volta ao patamar com o PI, integrador já carregado com a corrente do patamar, para não frear o rover. |
| `verify` | 1 s no patamar baixo e 1,2 s medindo o degrau de referência até o patamar alto, com `ESC_ACC` e `ESC_PREF` como na direção normal. Overshoot acima de 15% reduz a banda 25% e repete, no máximo 3 vezes, e só enquanto o overshoot estiver caindo. O que é gravado é o menor overshoot, não obrigatoriamente a última tentativa. |

Ganhos, ótimo simétrico com margem de fase perto de 46°:

- `wc = min(ESC_WT_BW, 1 / (2,65 T))`, e `T` é a mediana dos atrasos das rodas deste passo;
- `Kp = wc / Kv`;
- `Ti = 4 / wc`, `Ki = Kp / Ti`.

Kp e Ki têm de ser positivos e menores que `ESC_CUR` e `5 × ESC_CUR`. O integrador da verificação já começa com a corrente medida no patamar baixo, menos o feed-forward, para a roda não ficar presa no atrito seco e disparar depois.

No fim, cada roda guarda a média de frente e ré em `ESC_Pn`, `ESC_In`, `ESC_FFn`. A média das rodas vai para `ESC_P`, `ESC_I`, `ESC_FF`. `ESC_MODE` é gravado em 0 e `SCR_USER1` em 0. Uma roda presente num sentido e ausente no outro é corrida falha. Roda ausente nos dois sentidos é ignorada. A gravação na EEPROM sai uma por ciclo, para não estourar `SCR_VM_I_COUNT`.

O log `DJWT` leva, por roda sob teste: lado, sentido, fase (1 espera, 2 degrau, 3 verificação), rpm alvo, rpm, corrente e número da tentativa.

## Recusa de armar

O script pede um id de pré-armar e o reavalia a cada 100 ms.

Em PWM, o pré-armar passa.

Em CAN, falha com uma destas frases:

- `delete c620_can.lua from APM/scripts`, se o parâmetro `C620_CAN` ainda existe. A busca por nome ocorre no máximo uma vez por segundo.
- `ESC_LMASK needs ESCs on both sides`
- `ESC n not found (SERVOn is a motor)`, se a função do servo é 70, 73 ou 74 e esse ESC nunca respondeu
- `ESC n no feedback`
- `no ESC detected`
- `ESC_CAN=1 but CAN driver missing`, enquanto `CAN:get_device` falha. Sem o pedido de CAN, a ausência do driver não impede o armar.
- `ESC script fault; reboot`, depois de um erro de execução

## O que a estação de solo vê

Com `ESC_GCS_HZ` acima de 0 e CAN ativo, `NAMED_VALUE_FLOAT`:

| Nome | Valor |
| --- | --- |
| `ESCn_I` | Corrente medida, A, já com o sinal de `ESC_REV` |
| `ESCn_ICMD` | Corrente pedida pelo PI, A |
| `ESCn_RPM` | Rpm de roda |
| `ESC_TGT_L`, `ESC_TGT_R` | Rpm de referência do lado |

`ESC_OPT` bit 0 grava `DJIC` a cada 50 ms, por ESC detectado: rpm alvo, rpm de roda, corrente pedida, corrente medida com sinal, integrador, posição integrada do rotor em voltas de roda, temperatura.

A telemetria padrão de ESC (`esc_telem`) sai no máximo a cada 20 ms: temperatura em centésimos de grau, módulo da corrente e rpm do rotor. No laço, a primeira metade de cada 100 ms fica com a telemetria e a segunda com o log, para um ciclo não carregar os dois.

## Estado interno

### Cada ESC, `esc[1..8]`

`reset_esc` zera o slot. Slots 5 a 8 existem por causa do frame `0x1FF`; o controle e o log usam os quatro primeiros.

| Campo | Significado |
| --- | --- |
| `angle` | Último ângulo do rotor, 0 a 8191 |
| `have_angle` | Já chegou algum ângulo |
| `rotor_counts` | Contagens integradas pela rpm, só para o log |
| `rotor_rpm` | Rpm do rotor, como o C620 mandou |
| `current_raw` | Corrente crua, contagens de 20 A / 16384 |
| `temperature` | °C |
| `last_ms` | `millis` do último frame aceito |
| `detected` | Já apareceu ao menos um frame válido |
| `wheel_rpm` | Rpm de roda com sinal do veículo |
| `current_a` | Corrente pedida, A, sinal do veículo |
| `integrator` | Estado I, A |
| `last_error` | Erro anterior, rpm, para o D |
| `ref_rpm` | Referência depois da rampa de `ESC_ACC` |
| `ref_f` | Referência depois do pré-filtro |
| `target_rpm` | Alvo que entrou no PI neste ciclo |
| `saturated` | A conta passou de ±`ESC_CUR` a favor do erro |

### Cada lado, `side[1]` esquerdo e `side[2]` direito

| Campo | Significado |
| --- | --- |
| `integrator`, `last_error` | Resto do PI de lado, hoje não usado pelo laço |
| `rpm`, `rpm_mean` | Média das rpm das rodas detectadas desse lado |
| `rpm_lag` | Rpm da roda mais atrasada no sentido do comando |
| `diverge_rpm` | Limiar de divergência deste ciclo |
| `diverged` | O par de rodas abriu além do limiar |
| `ref_rpm` | Não é a referência por roda. A referência vive em `esc[i].ref_rpm` |
| `target_rpm` | Alvo do lado, rpm, publicado como `ESC_TGT_L/R` |
| `current` | Corrente da última roda escrita desse lado |
| `saturated` | Alguma roda do lado saturada neste ciclo |
| `goal` | Comando pedido, −1 a +1 |
| `interp_value` | Comando aplicado. Hoje copia `goal` no mesmo ciclo, sem interpolar entre amostras |

### Autotune, `tune`

Começa em `{ state = "idle" }`. Os outros campos nascem quando a corrida começa.

| Campo | Significado |
| --- | --- |
| `state` | `idle`, `rest`, `ff`, `step`, `recover`, `verify`, `done`, `fail` |
| `mode` | 4 ou 5, o seletor que iniciou. Mudou, aborta. |
| `only` | Roda sob teste no modo 5. `nil` no modo 4. |
| `rows` | Ganhos por sentido. `rows[1]` frente, `rows[2]` ré. Cada roda guarda `P`, `I`, `ff`. |
| `dir` | +1 frente, −1 ré |
| `ff_level` | 1 patamar baixo, 2 patamar alto |
| `t`, `count` | Tempo no estado e amostras da média |
| `sum_current`, `sum_rpm` | Acumuladores da média |
| `lo_i`, `lo_rpm` | Corrente e rpm médias do patamar baixo |
| `base`, `rpm` | Corrente e rpm médias do patamar alto |
| `tff` | Feed-forward medido, A por rpm |
| `step_i` | Amplitude do degrau, A |
| `fit` | Somas da reta de mínimos quadrados e o Kv e o atraso aceitos |
| `tkp`, `tki` | Ganhos candidatos deste sentido |
| `bw_scale` | Multiplicador da banda. Começa em 1 e cai 25% por overshoot |
| `tries` | Verificações já feitas neste sentido |
| `lag`, `wc` | Atraso mediano, s, e banda usada, rad/s |
| `peak`, `imax` | Maior rpm no sentido do teste e maior módulo de corrente na medição |
| `best_os`, `best_wc`, `bkp`, `bki` | Menor overshoot desta direção e os ganhos que o produziram |
| `kt` | Cópia dos ganhos cacheados com P, I e FF trocados pelos candidatos |
| `vphase` | `hold` ou `meas` dentro de `verify` |

### Outras variáveis

| Nome | Significado |
| --- | --- |
| `K` | Constantes. As de sintonia estão no meio do arquivo, junto do autotune. |
| `PARAM_SPEC` | Lista `{ índice, nome, padrão }` da tabela `ESC_`. A ordem é a ordem na EEPROM. Não inserir no meio. |
| `P` | Objetos `Parameter` já resolvidos, por nome curto (`P.CAN`, `P.P1`). |
| `params_ready` | A tabela foi lida e os objetos existem. |
| `latched_can` | Modo CAN confirmado desarmado. É o modo que o laço obedece, não o valor cru de `ESC_CAN` enquanto armado. |
| `driver` | Dispositivo CAN. `nil` até `CAN:get_device(25)` funcionar. |
| `auth_id` | Id de pré-armar. |
| `cmd_seq_seen`, `cmd_seq_ms` | Última sequência aceita e o instante dela. |
| `lost_mask` | Bits dos ESCs detectados que estão sem feedback. |
| `sat_ms` | Último instante em que algum lado saturou. |
| `slip_active` | Alguma roda está em patinação neste ciclo. |
| `slip_since[i]` | Desde quando a roda i está além de `ESC_SLIP`. |
| `M.dir[1..8]` | +1 ou −1, vindo de `ESC_REV`. |
| `M.side[1..8]` | 1 esquerdo, 2 direito. |
| `M.thr[1..8]` | O servo tem função 70, 73 ou 74. |
| `M.gains` | Cache dos ganhos. Reutilizado todo ciclo para o laço de 5 ms não alocar. |
| `M.scratch` | Somas, contagens e o vetor de corrente crua. Também reutilizado. |
| `M.latest` | Frame mais novo de cada ESC neste ciclo. |
| `M.tx`, `M.telem` | Frame de transmissão e pacote de telemetria, criados uma vez. |
| `M.save_q` | Fila de gravação na EEPROM, um item por ciclo. |
| `M.fault` | Texto do erro que travou o driver. `nil` enquanto o laço está são. |
| `diag` | Contagem de frames por id, outro id visto, e o pior intervalo entre laços na janela de 5 s. |

Constantes de tempo e de protocolo que mudam o comportamento se forem editadas:

| Constante | Valor | Efeito |
| --- | --- | --- |
| `LOOP_MS` | 5 | Período do laço, 200 Hz |
| `FEEDBACK_TIMEOUT_MS` | 100 | Sem frame, o ESC está morto e a corrente cai |
| `CMD_STALE_MS` | 500 | Sem sequência nova, comando do mixer vai a 0 |
| `GAINS_MS` | 50 | Período de releitura dos ganhos |
| `RAW_PER_AMP` | 16384/20 | Contagens do C620 por ampere |
| `RAW_MAX` | 16384 | Fundo de escala do int16 de corrente |
| `COUNTS_PER_REV` | 8192 | Contagens de ângulo do C620 |
| `MOVING_RPM` | 5 | Abaixo disso a roda conta como parada |
| `MAX_FRAMES` | 40 | Leituras CAN por ciclo |
| `SAT_HOLD_MS` | 100 | O bit de saturação fica aceso depois do evento |
| `SLIP_MS` | 100 | Patinação precisa durar isto para virar aviso |
| `HOLD_MODE` | 4 | Modo HOLD, exigido pelo autotune |
| `TUNE_H` | 7 | Razão do ótimo simétrico. `WC_LAG = sqrt(7) ≈ 2,65` |
| `STEP_S` | 0,25 | Duração máxima do degrau de corrente |
| `VERIFY_OS` | 0,15 | Overshoot aceito na verificação |
| `VERIFY_TRIES` | 3 | Tentativas de encolher a banda |

O primeiro agendamento espera 2 s (`return protected_update, 2000`), para a tabela de parâmetros e o barramento existirem. Falha de parâmetro ou driver ausente faz o laço voltar em 1 s. No erro travado, o ciclo de segurança é de 100 ms.

## Ordem de um ciclo são

`protected_update` chama `update` dentro de `pcall`.

1. Garantir a tabela `ESC_` e o vínculo com `SCR_USER1`, `SCR_USER5`, `RC1_TRIM`, `RC3_TRIM`.
2. Obter o id de pré-armar e o dispositivo CAN. Na primeira vez, instalar o filtro.
3. Atualizar máscaras de sentido, lado e servo.
4. Ler o cache de ganhos e aplicar `ESC_CAN` se estiver desarmado.
5. Com CAN ativo, segurar MAIN 1 a 4 em largura 0.
6. Drenar o CAN, rodar o controle, checar feedback perdido.
7. Avisar se `SCR_USER1` pede autotune com CAN desligado.
8. Publicar `ESC_STAT`, rodar o pré-armar, gravar um item da fila de EEPROM.
9. Na primeira metade de cada 100 ms, telemetria de ESC. Na segunda, log `DJIC`.
10. Enviar os `NAMED_VALUE_FLOAT` no ritmo de `ESC_GCS_HZ`.
11. Reagendar em 5 ms.

## Funções

Utilidade: `as_number`, `now_ms`, `constrain`, `finite`, `g`, `set_param`, `be_i16`, `put_i16`.

Parâmetros e mapas: `ensure_params`, `M.refresh_maps`, `M.queue_save`, `M.drain_saves`, `direction`, `side_of`, `mask_valid`, `is_throttle`, `read_gains`.

CAN e segurança: `kill_pwm`, `drain`, `accept_frame`, `send_group`, `apply_switch`, `M.check_health`, `M.prearm`, `M.safe_stop`, `M.report_rx`.

Comando e PI: `servo_side_norms`, `update_command`, `script_mixer_active`, `profile_wheel`, `current_window`, `wheel_pi`, `wheel_ti`, `prefilter`, `reset_command`, `reset_control`, `M.run_control`.

Autotune: `selector`, `tune_wanted`, `rc_centered`, `tune_reset`, `tune_fail`, `M.wheel_tune`, `M.tune_start_direction` (vale a segunda definição), `M.step_begin`, `M.step_close`, `M.step_gains`, `M.verify_begin`, `M.direction_done`, `M.tune_finish`, `M.save_wheel_gains`, `M.ff_slope`, `M.relay_amp`, `M.tune_levels`.

Aviso e publicação: `M.check_slip`, `M.publish_stat`, `M.write_log`, `M.push_telem`, `M.send_telem`, `M.send_gcs`, `M.warn_selector_without_can`.
