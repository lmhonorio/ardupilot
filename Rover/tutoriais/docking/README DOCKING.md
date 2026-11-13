> **ROS 2 Jazzy / Harmonic:** comandos migrados. Consulte [instalação e serviços](../ros2/ROS2_JAZZY.md). Resultados de precisão anteriores à migração são históricos e precisam ser repetidos no Harmonic.

# Aproximação da estação e observação do QR

**Sistema integrado:** o [Mission Manager](../mission_manager/TUTORIAL.md) coordena
missão, docking por LiDAR com QR opcional, recarga simulada e saída antes de AUTO.
Suas [medições e limitações](../mission_manager/TESTES.md) são registradas
separadamente dos ensaios visuais anteriores descritos abaixo.

Para iniciar os serviços e alternar entre entrada e saída, siga o
[tutorial de acionamento do docking no ROS 2](TUTORIAL_ENTRADA_SAIDA_ROS2.md).

**Atualização de 09/11/2026:** estratégia de aproximação, estimativa do QR e controle de giro revisados; velocidades de 0,25 m/s na aproximação e até 0,18 m/s na entrada. Seis entradas consecutivas aprovadas no Harmonic e 97 testes de software. Maior erro físico ao centro: 3.47 cm. Consulte [resultados, medições e limites](VALIDACAO_ALINHAMENTO_VELOCIDADE_ROS2.md).

## Executar os scripts no ROS 2

Use o ambiente **Jazzy / Python 3.12** selecionado por `ros2/setup.bash`.
Os comandos antigos com `/usr/bin/python3.8` ou `~/.pyenv/versions/3.8.10/`
não carregam as bibliotecas nativas do Jazzy. Para executar diretamente:

```bash
cd ~/ardupilot/Rover
source ros2/setup.bash
python python/docking/entrar_container.py --diagnostico
```

`aguardar_docking.py` também pode ser executado diretamente: ele encaminha a
inicialização para `ros2/service.sh docking-qr`, que carrega o ambiente e o
`pymavlink` instalado no projeto. Não é necessário instalar `pymavlink` no Python
do sistema. O comando recomendado para manter o monitor aberto é
`./ros2/service.sh docking-qr`.

O diagnóstico recebe imagem, `CameraInfo` e posições reais do pan/tilt, verifica
resolução, frame, calibração e timestamps e informa se algum QR está visível.
Ele encerra sem abrir conexão MAVLink e sem mover o rover ou a câmera.
`ROS 2 OK` confirma recepção, não a precisão do docking nem a visibilidade da estação.

Se o QR aparece na janela, mas o controlador não o reconhece, observe a mensagem:
“QR localizado (... px), mas conteúdo não decodificado” indica que o quadrado foi
detectado, mas ainda não foi possível ler o código. Com a câmera em 640×360, o QR
pode ocupar apenas cerca de 32 pixels de lado. O detector tenta ampliar a região
local e ajustar sua binarização, mantendo os cantos nas coordenadas originais
para calcular a pose. Ele só aceita o texto cadastrado da estação; ver o desenho
não substitui decodificá-lo. Essa recuperação foi testada com uma imagem recebida
do Harmonic, mas não garante leitura a qualquer distância ou ângulo.
Após atualizar o código, encerre o monitor antigo com Ctrl+C, aguarde a saída do
controlador e inicie novamente `./ros2/service.sh docking-qr`. Não abra dois
controladores na mesma porta MAVProxy.

Com Gazebo, SITL e MAVProxy ativos, os atalhos abaixo carregam o ambiente automaticamente.
Execute **uma rotina de controle por vez**, em terminal separado:

| Objetivo | Comando dentro de `~/ardupilot/Rover` |
| --- | --- |
| Diagnosticar câmera e juntas | `./ros2/service.sh docking-qr --diagnostico` |
| Observar QR continuamente, sem comandar | `./ros2/service.sh observar-qr` |
| Aguardar entrada em GUIDED, armado e a menos de 8 m | `./ros2/service.sh docking-qr` |
| Sair de ré e girar 180°, estando em HOLD dentro da estação | `./ros2/service.sh sair-qr` |
| Diagnosticar nuvem local e odometria, sem comandar | `./ros2/service.sh docking-lidar --diagnostico` |
| Aguardar ativação da entrada por LiDAR | `./ros2/service.sh docking-lidar` |
| Sair usando LiDAR | `./ros2/service.sh sair-lidar` |

Para LiDAR, inicie antes `./ros2/service.sh fastlio` e `./ros2/service.sh mapa`,
cada um em seu terminal. A rotina utiliza `/rover_slam_3d/local_cloud` e
`/fast_lio/odometry`; o mapa global acumulado é para visualização e não substitui
a nuvem local atual. `--cloud` e `--odom` permitem selecionar outros tópicos
compatíveis. A odometria precisa ser contínua, com frames `camera_init -> body`,
sem saltos de fechamento de laços. Trocar apenas o nome do tópico de outro SLAM
não converte frames nem calibra extrínsecos.

Nas rotinas visuais, `--namespace /rover_argo_N1/Instance1` seleciona câmera,
CameraInfo e juntas. O monitor repassa esse argumento ao controlador de entrada.
O namespace ROS é independente de `--sysid` e `--mavproxy`; mantenha os três
apontando para o mesmo rover. Os extrínsecos continuam sendo os do modelo Samsung
com pan/tilt deste cenário. O transporte dos comandos continua via MAVProxy UDP
14551; ROS 2 fornece os sensores. As rotinas continuam limitadas ao SITL.

Se aparecer “sem mensagens recentes”, confira se o Gazebo está executando e se
FAST-LIO/mapa não foram suspensos com **Ctrl+Z**. `jobs -l` no terminal original
mostra os processos suspensos; `fg` retoma o trabalho selecionado. **Ctrl+C**
encerra o serviço. Se reiniciou o Gazebo, encerre e inicie novamente FAST-LIO,
mapa e docking para descartar referências anteriores. A rotina LiDAR rejeita
um recuo do relógio `/clock`, em vez de continuar usando a pose antiga.

Validação desta revisão: 97 testes de software e seis entradas completas no
Harmonic. Consulte as [condições e resultados medidos](VALIDACAO_ALINHAMENTO_VELOCIDADE_ROS2.md).
A saída não foi revalidada nesta revisão. Consulte também [validação ROS 2](../ros2/VALIDACAO_ROS2.md).

Para a alternativa sem câmera e sem QR, consulte [Entrada e saída com MID360](DOCKING_LIDAR.md).

> Para acionar o docking sem depender de missão, use a seção [Espera automática para docking](#espera-automática-para-docking-em-guided). O comando `volta_e_docking.py` abaixo carrega outra missão; não o execute junto com o monitor.

## Volta e entrada automática no contêiner

A missão [`volta_docking.waypoints`](../../simulacoes/docking/volta_docking.waypoints), para o mundo `gravel_plane`, percorre aproximadamente os pontos `(-1,-2)`, `(-1,2)`, `(2,2)` e `(2,-2)` e termina em `(3.2,-2)` metros no referencial do Gazebo. A última reta aponta para a abertura do contêiner, em `x=4,5 m`. O arquivo pressupõe a origem SITL `SEParnaiba` em `-3.12315, -41.764495`; confira o mundo, a origem e os obstáculos antes de executar.

1. Inicie o Gazebo, o SITL e os demais componentes conforme o [passo a passo](../simulacao/PASSO_A_PASSO_ROS_FAST_LIO.md), com a câmera Samsung habilitada e o rover fora do contêiner, parado e desarmado. Aguarde GPS, EKF e as verificações PreArm ficarem prontos.
2. Confirme que o MAVProxy está ativo, com saída `udp:127.0.0.1:14551` e `set mavfwd true`. O `sim_1_gzrover.sh` já configura essa saída. Abra o QGroundControl para acompanhar.
3. Em outro terminal, execute:

```bash
source ~/ardupilot/Rover/ros2/setup.bash
cd ~/ardupilot/Rover
python python/docking/volta_e_docking.py
```

O programa salva a missão anterior em `python/run/docking/volta_<data>/missao_anterior.json`, substitui a missão no veículo, arma e inicia AUTO. Após concluir a aproximação e estabilizar em HOLD, chama a entrada visual em GUIDED descrita abaixo. A entrada somente começa se o QR correto estiver visível e o alinhamento satisfizer os limites do controlador. Para interromper, use Ctrl+C no terminal ou `mode hold` no MAVProxy; acompanhe as mensagens e a restauração dos parâmetros de desvio.

Esta sequência completa ainda não foi validada em movimento: um ensaio isolado foi interrompido antes de armar porque o estado PreArm não ficou saudável. O arquivo de missão anterior é um registro JSON, não um arquivo WPL diretamente carregável. A entrada visual, isoladamente, foi ensaiada conforme [resultado e condições](RESULTADO_ENTRADA.md). Este programa é exclusivo para SITL; o estado PreArm e os sensores devem estar saudáveis, sem desabilitar verificações para forçar a execução.

A missão [`aproximacao_qr.waypoints`](../../simulacoes/docking/aproximacao_qr.waypoints) foi gerada para `gravel_plane`, com origem GPS do SITL `SEParnaiba` em -3.12315, -41.764495. Os dois destinos no Gazebo são aproximadamente `(x=2, y=-2)` e `(x=3, y=-2)` m: o último fica 1,5 m antes da entrada, que está em x=4,5 m. O trecho final aponta para o fundo da estação, mas a trajetória, a orientação final e a precisão GPS **ainda não foram validadas**; raio de waypoint e desvio podem fazer o rover parar fora do alinhamento.

Para carregar a missão no MAVProxy após restaurar a simulação, use `wp load /home/grin/ardupilot/Rover/simulacoes/docking/aproximacao_qr.waypoints`. Isso substitui a missão no veículo; salve a anterior antes caso precise dela. `mode hold`, `wp set 0` e `mode auto` iniciam a missão se o rover estiver armado e as verificações de pré-armamento estiverem satisfeitas. Confirme no QGroundControl e acompanhe no Gazebo.

Para testar a leitura do QR pela câmera Samsung:

```bash
source ~/ardupilot/Rover/ros2/setup.bash
cd ~/ardupilot/Rover
python python/docking/observar_qr.py
```

O observador é somente leitura: identifica `ROVER_DOCKING_STATION_01`, apresenta distância aproximada e ângulo na imagem; não muda modo e não movimenta o rover. A estimativa assume que o QR está aproximadamente de frente e usa o tamanho efetivo do padrão impresso. Deve ser calibrada para ângulos e distâncias reais.

A leitura pela câmera móvel e a aproximação no Harmonic estão documentadas no relatório de validação indicado no início deste arquivo. Resultados de versões anteriores ao ROS 2 permanecem nos relatórios históricos.

## Espera automática para docking em GUIDED

Após iniciar Gazebo e MAVProxy, deixe este programa aberto em um terminal separado:

```bash
source ~/ardupilot/Rover/ros2/setup.bash
cd ~/ardupilot/Rover
python python/docking/entrar_container.py
```

O programa escuta a saída local UDP 14551 do MAVProxy sem consultar, criar, enviar ou alterar missões. Ele tenta iniciar a entrada quando houver telemetria recente do SITL, rover armado em modo **GUIDED**, GPS 3D e distância horizontal **menor que 8 m do centro** de uma estação cadastrada. Não é necessário chegar a qualquer waypoint ou ter uma missão carregada.

As estações estão cadastradas em [`estacoes_recarga.txt`](../../estacoes_recarga.txt).
Cada linha informa nome, dois pontos GPS a 4 m e 2 m **da abertura**, conteúdo
do QR, dimensões internas e distância do centro ao QR. O rumo segue P4 → P2;
a abertura e o centro são calculados prolongando esse eixo. Consulte o
[formato do cadastro](CADASTRO_ESTACOES.md). As entradas por QR e LiDAR escolhem
a estação mais próxima; `--estacao NOME` restringe a seleção. Reinicie o
controlador depois de editar o arquivo. Esses pontos definem a referência,
sem criar itens de missão. O GPS continua sendo uma referência aproximada.

Antes de mudar o modo, deixe o rover **parado**. Se o QR não estiver visível, o controlador busca a estação girando em sua direção pelo GPS cadastrado. O limite de 8 m apenas autoriza a tentativa; a detecção do QR e os demais limites ainda precisam ser satisfeitos. O controlador solicita HOLD durante as verificações e volta a GUIDED. Ele converge para um ponto no eixo central fora da abertura, para para corrigir o rumo e confirma alinhamento antes de entrar; ao entrar na região final, solicita HOLD imediatamente com margem de frenagem baseada na velocidade e só desarma depois de confirmar parada e verificar novamente a pose visual. Nessa região não faz novas manobras; se perder o QR, permanece em HOLD. Se uma tentativa falhar, leia o motivo; é preciso sair de GUIDED e ativá-lo novamente para outra tentativa. Depois de terminar, o programa volta a aguardar. Ctrl+C encerra a espera ou interrompe a entrada em andamento.

O monitor observa o **modo efetivo GUIDED** no heartbeat. Nesta versão do Rover, o item de missão `MAV_CMD_NAV_GUIDED_ENABLE` inicia um submodo guiado *dentro de AUTO* e não equivale necessariamente a uma troca do modo informado no heartbeat; ele, sozinho, não foi integrado a este gatilho. Nossa explicação anterior que tratava os dois como equivalentes estava incorreta. Uma troca efetiva de modo pode ser comandada pelo QGroundControl ou MAVProxy depois que o rover parar.

A ativação e o desarmamento têm testes de software; consulte o relatório de validação no início deste documento para os ensaios em movimento desta revisão. As tolerâncias são de pose estimada pela câmera e não garantem precisão física. O processo é exclusivo para SITL e não deve compartilhar a porta 14551 com outro programa, inclusive `volta_e_docking.py`.

## Entrada visual imediata experimental

Depois que o rover estiver parado, com o QR da estação visível, execute em outro terminal:

```bash
source ~/ardupilot/Rover/ros2/setup.bash
cd ~/ardupilot/Rover
python python/docking/entrar_container.py --uma-vez
```

O programa é para este SITL e usa UDP 14551 via MAVProxy. Ele exige um rover armado, parado, em HOLD, AUTO ou GUIDED, telemetria SITL e câmera reconhecendo o QR correto. O controlador calcula a aproximação ao eixo a partir da pose estimada do QR; durante a entrada estreita usa limites mais restritos de alinhamento. O sistema inicializa pan em zero e tilt em -0,30 rad, confirma as juntas e passa a acompanhar o QR com ambos os eixos. A pose usa os ângulos medidos das juntas no instante da imagem, incluindo a rotação do rover entre a exposição e a telemetria. Se esses requisitos não forem satisfeitos, ele interrompe antes de comandar o movimento.

A posição relativa é calculada pela câmera e pelas dimensões do QR. O alvo nominal é o centro do contêiner, com distância de 0,499 m em relação ao QR, conforme a geometria do modelo Harmonic; o programa transmite `LANDING_TARGET` em BODY_FRD ao ArduPilot pelo MAVProxy e envia setpoints de velocidade e giro em GUIDED. Esses dados não são fundidos automaticamente no EKF. O limite padrão de aproximação é **0,25 m/s**, com limite de **0,18 m/s durante a entrada**, aceleração gradual e redução até cerca de 0,035 m/s nos centímetros finais. Use `--velocidade` para reduzir o limite; valores maiores que 0,25 m/s não são aceitos. Esses limites são próprios do docking; `WP_SPEED=1.5` continua sendo a velocidade nominal de missão. O limite padrão de execução é 300 s, configurável com `--timeout` até 600 s. Ao iniciar, a rotina salva `OA_TYPE` e `AVOID_ENABLE` em `python/run/docking/<data>/parametros_anteriores.txt`. Ela solicita desabilitar o BendyRuler e a proximidade do desvio simples ao iniciar a entrada, mantendo o bit da cerca; a aplicação de `OA_TYPE` pode exigir reinício no ArduPilot, portanto confira o comportamento efetivo no SITL. Se o QR não for reconhecido, o controlador gira em direção ao centro da estação usando o GPS e o rumo. Com candidato visível, pode executar a aproximação curta descrita abaixo para recuperar a leitura; após decodificar o QR cadastrado, retoma o controle visual. Se a imagem, GPS ou rumo não estiverem confiáveis, aguarda sem girar; a busca pode continuar sem limite de tempo. Ao terminar, detectar perda de telemetria ou alinhamento fora da tolerância, solicita HOLD e tenta restaurar os parâmetros salvos. Se houver falha na restauração, consulte esse arquivo e os erros impressos.

Ctrl+C solicita HOLD e restauração. Uma falha abrupta do computador, encerramento forçado ou perda prolongada da conexão pode impedir a restauração; confira `param show OA_TYPE` e `param show AVOID_ENABLE` antes de voltar à navegação. Não use em um veículo real.

A geometria tem testes com imagens sintéticas e com quadros recebidos do
Harmonic. Os ensaios em movimento, suas condições iniciais e os erros físicos
medidos estão no relatório de validação indicado no início deste arquivo.
O trajeto local de aproximação pressupõe área livre ao redor da estação; a
pose visual não verifica obstáculos nessa área nem confirma contato elétrico
de carregamento. A rotina de saída e a navegação por LiDAR têm validações próprias.

A finalização registra amostras da pose e velocidade em `python/run/docking/<data>/pose_visual.jsonl`. A tolerância após a parada é de 6 cm longitudinal, 8 cm lateral e 6 graus; se a pose não puder ser confirmada dentro desses limites, permanece em HOLD e armado. A frenagem considera velocidade medida, latência e desaceleração. O resultado físico é verificado nos ensaios por uma medição independente do Gazebo, sem fornecer essa posição ao controlador.

## Estimativa visual e controle do alinhamento

O QR decodificado é comparado ao padrão completo da textura cadastrada para
refinar seus cantos. A estimativa considera a parede vertical e a posição real
do pan/tilt. O rumo cadastrado fornece uma referência inicial fraca; as imagens
confirmam e corrigem a orientação. Amostras inconsistentes param o movimento
até a confirmação por novas imagens.

A aproximação alcança um ponto no eixo a cerca de 1,2 m do centro, ainda fora
da abertura. O rover interrompe o avanço para corrigir o rumo e aguarda o giro
cessar antes de acelerar novamente: pequenos comandos de giro durante o avanço
não venceram de forma confiável o atrito lateral do modelo. O pan/tilt mantém o
QR visível nessas manobras. Na entrada, a referência angular converge para o
eixo das paredes. Se estiver próximo demais e deslocado,
pode recuar lentamente para criar espaço antes da abertura; isso exige área
livre atrás e pose visual válida.

Durante a tentativa, a rotina salva os valores atuais e aplica este perfil de
direção para baixa velocidade: `ATC_STR_RAT_FF=0.7`, `ATC_STR_RAT_P=0.7` e
`ATC_STR_RAT_I=0`. Isso evita acumular correção integral enquanto o atrito impede
o giro e carregá-la para o próximo avanço. Ao terminar ou receber Ctrl+C, ela
solicita HOLD e restaura os três valores, além dos parâmetros de desvio que
alterou. Os valores anteriores ficam em
`python/run/docking/<data>/parametros_anteriores.txt`; consulte esse arquivo se
o processo for encerrado à força. O perfil de navegação nos arquivos `.param`
não é alterado por essa rotina.

Esses ajustes foram feitos para a entrada por QR no contêiner do Harmonic. O
controle por LiDAR mantém seus parâmetros e sua lógica anteriores em
`python/docking_lidar/alinhamento.py`. Não há uso da posição verdadeira do
Gazebo para orientar a entrada visual: ela serve apenas para verificar os
ensaios. A leitura do QR é obrigatória para entrar e confirmar a parada.

## Aproximação curta quando o QR não decodifica

O controlador pode avançar para aumentar o QR na imagem, antes da fase de entrada,
quando há um candidato visível mas o conteúdo ainda não foi decodificado. Exige:

- Erro de rumo até o centro GPS cadastrado **menor que 5°**.
- Direção do candidato a QR a menos de **5° da frente do rover**, considerando o pan/tilt medido.
- Imagem, GPS, rumo, velocidade e giro atuais; giro abaixo de 0,05 rad/s.
- Alinhamento mantido por **0,8 s** e distância ao centro da estação **maior que 2 m**.

A velocidade é **0,04 m/s**, limitada a **0,40 m de deslocamento GPS desde o início
da busca** ou **12 s acumulados de comando de avanço**, o que ocorrer primeiro.
O orçamento não é renovado ao perder/reencontrar o QR. Em simulação lenta, o limite
de tempo pode encerrar a aproximação antes dos 40 cm. Se o limite for atingido sem
leitura, o avanço fica bloqueado até iniciar outra tentativa.

Se o GPS já aponta para a estação, mas o candidato continua fora do eixo, o giro
ajusta sua centralização antes de avançar. Perda da imagem, do candidato ou das
condições de alinhamento cancela o avanço imediatamente. Um código decodificado
com outro identificador não autoriza aproximação. A busca não desabilita o desvio
nem avança dentro do contêiner. A entrada e a pose final continuam exigindo o QR
cadastrado decodificado; alinhamento GPS para o centro não comprova alinhamento
com o eixo da abertura.

Os limites ficam em `python/docking/busca_qr.py`, classe `AproximacaoBusca`.
Foram testados os limites angular, de distância e de tempo, a perda de dados e a
centralização. A aproximação nova ainda não foi validada em movimento no Gazebo.
Reinicie apenas o monitor para carregar o código atualizado; ao encerrar com Ctrl+C,
aguarde HOLD e volte a GUIDED somente com o rover parado e as condições de ativação
satisfeitas.

## Saída do contêiner de ré

Com Gazebo, câmera e MAVProxy ativos, deixe o rover parado em **HOLD**, dentro da estação e de frente para o QR cadastrado. Se o monitor de entrada estiver aberto na porta UDP 14551, encerre-o com Ctrl+C antes de iniciar a saída; os dois programas não podem ocupar essa porta ao mesmo tempo. Em outro terminal:

```bash
source ~/ardupilot/Rover/ros2/setup.bash
cd ~/ardupilot/Rover
python python/docking/sair_container.py
```

A rotina exige telemetria SITL, leitura do QR da estação `container_gravel_plane` e uma pose visual compatível com o interior e o alinhamento do contêiner. Se estiver desarmado, solicita armamento normal; se o ArduPilot recusar, não prossegue. Em GUIDED, recua seguindo o QR e usa a variação da posição visual para medir os 2 m. Em seguida solicita HOLD e confirma parada, distância entre 1,85 e 2,15 m e alinhamento antes de girar. O giro de 180° usa o yaw do ArduPilot; ao terminar, solicita HOLD e desarma. O programa não altera a missão.

Para outro cadastro, use `--estacao NOME`. `--velocidade` admite até 0,12 m/s e `--timeout` ajusta o tempo de cada etapa. Ele salva os valores anteriores de `OA_TYPE` e `AVOID_ENABLE` em `python/run/docking/saida_<data>/parametros_anteriores.json`, solicita desativar o desvio para a passagem estreita e tenta restaurar os valores ao terminar ou interromper. `OA_TYPE` pode exigir reinicialização para afetar o planejador: confirme o comportamento da configuração ativa. A rotina não valida obstáculos atrás do rover; mantenha a área de recuo e giro livre.

Se o QR sumir durante o recuo ou o alinhamento ultrapassar o limite, solicita HOLD sem iniciar o giro. Se o modo for alterado externamente, interrompe a rotina. Ctrl+C também interrompe e tenta parar o rover. A estimativa depende da câmera e da calibração simuladas; os testes realizados cobrem a lógica geométrica, não uma saída completa em movimento no Gazebo. Não use no rover real sem adaptação e validação.
