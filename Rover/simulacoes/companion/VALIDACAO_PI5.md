# Instalação e validação — Raspberry Pi 5

Ensaios em 12/11/2026, destino `rover@192.168.0.167`.
Tutoriais: [SITL/Gazebo](INSTALACAO_PI5_SIMULACAO.md) e
[rover físico](INSTALACAO_PI5_ROVER_REAL.md).

## Ambiente e arquitetura

Raspberry Pi 5 Model B Rev 1.1, 16 GB RAM, Ubuntu 24.04.5 ARM64,
cartão de 16 GB. Docker 29.9.3, Compose 5.7.1 e armazenamento `overlay2`.
ROS 2 Jazzy, Livox SDK2/driver, FAST-LIO, mapa 3D, docking LiDAR e gerenciador
de missão estão na imagem `rover-companion:jazzy`. ROS não foi instalado no
host da Raspberry: a execução é pelo Docker.

Gazebo Harmonic, SITL/ArduRover, Lua, MAVProxy e RViz ficam no computador.
A missão é solicitada pelo computador e executada pelo gerenciador na Pi.
O perfil ensaiado mantém QR e desvio de obstáculos desativados. O código QR
está incluído na imagem, mas estes ciclos não validam docking visual.

Referência do repositório: `747dfbf7b18e8d84937c89a592a632f38e91b406`,
com as alterações de instalação/endereço e documentação desta tarefa.
Não foram alterados Lua, PIDs, tolerâncias de docking ou limites de segurança.
Não foram alterados endereços IP ou rotas.

Imagem final ARM64, aproximadamente 3,85 GB:
`sha256:0c7c67ad94fdd43e0e2e30846cd9f40867660a53a3d68d28a3d34aaa01109cd6`.
Os ciclos usaram a imagem anterior com o mesmo código de controle; a última
reconstrução removeu um XML experimental de transporte que não era utilizado.
O teste de dependências foi repetido na imagem final e passou. O perfil real
recusou inicializar sem calibração, conforme esperado (`release-real-guard.log`).

## Verificações de software

| Verificação | Resultado | Evidência |
| --- | --- | --- |
| Regressão Python no computador | 229 testes passaram | `regression.log` |
| Separação dos perfis no ARM64 | 8 testes passaram | `profiles-arm64.log` |
| Imagem ARM64: interfaces ROS, MAVLink, OpenCV/QR, bibliotecas nativas | Passou | `image-smoke.log`, `image-smoke-release.log` |
| Adaptador nativo do sensor simulado | Passou com strides 1, 8 e 16; campos/tempos/padding | `simulation_adapter.log` |
| Retenção do mapa | Passou: dados antigos invalidam nuvem local; reversão do relógio limpa mapa | `map_retention.log` |
| Transporte interno de nuvens completas | 20/20 mensagens de aproximadamente 1,4 MiB íntegras | `large_cloud_transport.log` |

Arquivos de evidência no computador:
`Rover/python/run/companion_validation/20261112_pi5/`.
Essa pasta de execução é ignorada pelo Git; arquive-a separadamente para preservar
os dados brutos. Os logs do gerenciador ficam também na Pi, em
`~/rover_companion/data/runs/20261112_125056/manager/events.jsonl`.

## Ensaios completos de missão

Missão `Missions/new_argo.plan`, seis waypoints, mundo `gravel_plane`,
MID360 e câmera Samsung ativos, Insta360/Hokuyo desligados.
O validador `companion/tests/validate_mission.py` solicita a missão pelo serviço
ROS, observa as transições e compara a posição final com a pose independente
do Gazebo. Não usa apenas a estimativa do docking para declarar sucesso.

Critérios: chegar a `RECARGA_CONCLUIDA`, estar dentro, HOLD, desarmado,
bateria SITL em 100%, erro longitudinal e lateral menor que 8 cm e
erro de orientação menor que 6°. Esses critérios são de validação do ensaio,
não uma garantia de precisão em hardware físico.

| Ciclo | Condição inicial | Resultado |
| --- | --- | --- |
| 1 | Fora do container, HOLD/desarmado | **Passou**: missão, docking, desarmamento e recarga a 100%, em 323,24 s de relógio real |
| 2 | Dentro, após recarga do ciclo 1 | **Falhou na saída**: `Saida perdeu alinhamento/odometria`; foi para HOLD e não iniciou a missão |

No ciclo 1, erros finais medidos pelo Gazebo: longitudinal −1,74 cm,
lateral −2,35 cm e orientação −0,21°. Evidência: `cycle1/result.json`.

No ciclo 2, a última amostra registrada antes da falha estimava 2,025 m de ré
pelo FAST-LIO e 2,379 m pela geometria da estação, com desvio lateral LIO
−11,74 cm. A proteção de saída verifica limite lateral de 12 cm, orientação
12°, direção e saltos de odometria. O evento de falha não registra qual
comparação disparou; a aproximação ao limite lateral é evidência, não uma
confirmação exclusiva da causa. Também houve esperas por varreduras recentes.
Não foram ampliadas tolerâncias para fazer o teste passar. Evidências:
`cycle2/result.json` e `events.jsonl`.

**A saída automática completa de 3 m não foi validada nesta instalação.**
O ciclo 1 comprova entrada e recarga nesse ensaio; não basta para declarar
confiável a sequência repetida saída → missão → entrada. A divergência entre
LIO e geometria e a irregularidade de entrega precisam ser tratadas antes
de depender dessa sequência sem supervisão.

## Rede e desempenho

O gargalo observado foi a entrega irregular de nuvens pela rede, não falta
de RAM na Pi. Aumentar somente os sockets UDP para 8 MiB não resolveu tudo:
o orçamento separado de remontagem IP ainda estava em 4 MiB, com retenção
de fragmentos por 30 s. Houve crescimento de falhas de remontagem.

O script `configure_dds_buffers.sh` agora também configura pelo menos 64 MiB
para remontagem e retenção máxima de 3 s. Foi aplicado na Pi. No computador,
o ajuste adicional depende de sudo local; os ensaios aqui foram feitos ainda
com os valores de remontagem de 4 MiB/30 s no computador. Os sockets já tinham
8 MiB. As configurações anteriores são preservadas em `/var/lib/rover-companion`.

Foram experimentados fragmentos DDS menores e transporte TCP. Embora testes
isolados TCP transportassem nuvens completas (20/20 PC→Pi, 18/20 Pi→PC),
a inicialização conjunta de relógio/odometria não ficou estável. Essas opções
foram removidas. A configuração entregue usa UDP entre máquinas e memória
compartilhada dentro do container.

O ensaio foi realizado com `real_time_factor=0.5` (medido 0,50074).
Com 1,0 ainda houve entrega irregular: não está validado para operação remota
em tempo real nesta rede. Em uma janela de 12 s com fator 0,5, a Pi recebeu
29 nuvens de 46.080 pontos, 32 odometrias e 27 nuvens locais; nenhuma dessas
nuvens estava vazia. A idade máxima nessa janela foi 0,268 s para o sensor e
0,288 s para odometria/nuvem local, em tempo ROS. Houve esperas por dados
recentes em outros momentos, portanto essa janela não prova ausência de perdas.

Uma amostra durante a missão: CPU do container 55,2% (100% representa um núcleo),
RAM 656,1 MiB, temperatura da Pi 66,65 °C e aproximadamente 6,2 GiB livres no
cartão. São amostras, não máximos nem um teste prolongado de carga.

## Limites para o rover real

O perfil físico permanece com `hardware_configured: false`.
Não houve teste com autopiloto físico, MID360 físico, motores ou carregador.
A imagem contém os componentes, mas faltam identificação da placa/serial,
baudrate, IPs do MID360, extrínsecos, envelope e estações reais.
O carregador físico precisa de integração própria; o perfil real não executa
o reset de bateria usado na recarga simulada.

Os perfis isolam relógio, aquisição, MAVLink e recarga, preservando as interfaces
de missão/docking. Isso permite reaproveitar a imagem; não substitui o
comissionamento do veículo real descrito no segundo tutorial.

## Estado deixado após os testes

SITL confirmado em HOLD e desarmado após a falha de saída. O container foi
encerrado com `docker compose down`; RViz, SITL/MAVProxy e Gazebo foram
encerrados. A imagem, configurações, missões e logs permanecem instalados.
`restart: 'no'` evita iniciar controle automaticamente ao ligar a Pi.
O diagnóstico da falha foi preservado; reiniciar serviços não comprova sua
resolução. Para a falha registrada, confirme a posição real no cenário e
a qualidade dos sensores antes de solicitar reconciliação ou outra missão.
