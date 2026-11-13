# Verificação da alocação — 27/10/2026

## Sintoma e diagnóstico

O registro da execução do usuário mostra AUTO, aproximadamente 44% de
throttle, velocidade de cerca de 0,01–0,06 m/s e saídas C1=2000 e
C3≈1429 µs. Portanto, o Lua estava entregando comandos aos dois lados.
Throttle no HUD é uma saída do controlador, não uma medida de velocidade.

Um instante do log fornece throttle=0,4383198547 e steering=0,585625271.
A mistura L=throttle+steering e R=throttle−steering, normalizada pelo maior
módulo quando há saturação, resulta em L=1 e R≈−0,144. Com trim de 1500 µs
e extremos de 1000/2000 µs, isso corresponde a 2000/1428 µs. Um lado está
no máximo à frente e o outro em ré, tentando executar a curva. A taxa de
giro desejada era aproximadamente 13,8 °/s, mas a medida estava perto de
zero. A posição/velocidade física simulada também indicava pouco movimento.

O caso numérico foi incluído em `../tests/test_control_allocation.lua`.
Os 27 testes de Lua passaram. Não foi necessário alterar novamente a
fórmula de mistura em `functions.lua` nesta verificação.

Ensaios em MANUAL, comunicando por MAVProxy e alternando BATT_SOC_ALLOC
entre 1 e 0, produziram saídas e movimentos semelhantes em comandos curtos.
Em campo livre, com o modelo anterior, ambos também conseguiram mover e
girar com o comando de 44%/59%. Isso afasta a hipótese de que a mistura Lua
esteja simplesmente anulando o throttle, mas **não isola a causa exata da
parada original**. Contatos com obstáculos, estado físico e resposta das
juntas precisam ser considerados se o sintoma reaparecer nesse local.

## Ajuste aplicado no Gazebo

O plugin ArduPilot usa servos de velocidade que aplicam torque às juntas.
Foi alterado somente o modelo local `models/rover_mid360/model.sdf`:

| Configuração, nas quatro rodas | Antes | Depois |
| --- | --- | --- |
| P | 1 | 10 |
| I | 0 | 0 |
| D | 0 | 0,02 |
| Limite de torque por junta | ±10 N m | ±20 N m |
| useForce | true implícito | true explícito |
| rotorVelocitySlowdownSim | 1 efetivo | 1 explícito |

O multiplicador 70, offset −0,5, canais e sentido das rodas foram mantidos.
O carregador do plugin usa slowdown=1 quando o campo está ausente; não
havia redução efetiva de dez vezes na configuração anterior. Os arquivos
de `/home/grin/ardupilot_gazebo` não foram modificados.

Esses ganhos pertencem aos atuadores da simulação, não aos controladores
ATC_* nem ao hardware real. O Gazebo precisa reiniciar para carregar o SDF.
Para o funcionamento de servos de velocidade por força/torque, consulte o
[tutorial oficial do Gazebo Classic](https://classic.gazebosim.org/tutorials?tut=set_velocity).

## Ensaio com o ajuste

Foi usado Python 3.8, ArduRover compilado, MAVProxy e BATT_SOC_ALLOC=1.
O mundo temporário manteve chão e rover, retirando os obstáculos para
separar navegação e colisões. O mundo de produção não foi alterado.
O leitor de câmera/LiDAR não foi iniciado nesse ensaio.

Comando de missão, após iniciar os componentes nesse mundo de diagnóstico:

```bash
./mission_tune.sh --radius 3 --speed 0.4 --wp-radius 0.2 \
  --circle-points 12 --execute --output-dir DIRETORIO_NOVO
```

O percurso tem círculo de raio 3 m, aproximado por 12 segmentos, seguido de
quadrado circunscrito de lado 6 m. Inclui aproximação, conexão e espera final.
O perfil habitual de 36 segmentos não foi usado nesse ensaio.

| Medida | Resultado |
| --- | --- |
| Missão | Concluída |
| Velocidade de referência | 0,4 m/s |
| Pico de velocidade horizontal | 0,594 m/s |
| Erro RMS de velocidade, amostras PID | 0,050 m/s |
| Erro RMS de trajetória geral | 0,321 m |
| Erro RMS de trajetória no círculo | 0,417 m |
| Erro RMS de trajetória no quadrado | 0,238 m |
| Erro radial RMS no círculo | 0,376 m |

Os ganhos ATC_SPEED_P/I=0,15/0,20 e ATC_STR_RAT_P/I=0,20/0,20 foram
mantidos. O ensaio usou verificações normais de armamento e limite de
interrupção de 1,5 m/s do utilitário. Ao terminar, os parâmetros temporários
foram restaurados e o rover foi colocado em HOLD e desarmado. A missão
anterior de sete itens foi reenviada e conferida por leitura MAVLink.
Os processos de diagnóstico foram encerrados.

Um ensaio anterior em MANUAL, após o ajuste, foi interrompido pelo limite
de 3,5 m/s do diagnóstico ao aplicar 44% de throttle e 59% de steering.
Portanto, o modelo não limita velocidade em MANUAL a 0,4 m/s; o resultado
de velocidade acima refere-se à missão em controle fechado.

Não houve missão equivalente antes/depois, nas mesmas condições. Estes
resultados demonstram uma execução funcional após o ajuste; não demonstram
ganhos ótimos, precisão de 20 cm ou resolução definitiva da parada original
no cenário com obstáculos. Não houve validação no rover real.

## Evidências locais e próxima execução

Os registros estão em
`run/allocation_check_20261027_173149/` (diretório ignorado pelo Git):

- `user_mav.tlog` e `user_ardurover.log`: cópias anteriores aos ensaios.
- `model_before.sdf` e `model_after.sdf`: configurações comparadas.
- `field_before/` e `field_after/`: comandos em MANUAL, incluindo a interrupção.
- `closed_loop_after/report.html` e `report.json`: métricas da missão concluída.
- `restore/mission_restored.json` e `final_state.json`: missão e estado final.

Para voltar ao mundo com obstáculos e abrir a interface:

```bash
cd /home/grin/ardupilot/Rover
./start_gz_simple.sh --lua --no-rebuild
```

Se reaparecer a parada, comparar o instante com a posição no mundo, possíveis
contatos, C1/C3, velocidade física e taxa de giro, além do throttle do HUD.
