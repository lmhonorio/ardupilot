> Código deste componente: `Rover/mid360/fast_lio_raspberry/`. Execute os comandos a partir de `~/ardupilot/Rover`; caminhos `run/` e `ws/` deste documento se referem ao diretório do código.

# Resultados — 13/11/2026

Ensaio com MID360 físico parado, Raspberry Pi 5 e PC na mesma rede. FAST-LIO
executado na Pi, visualização pelo PC. Os números abaixo medem funcionamento e
estabilidade estática; não medem precisão absoluta nem desempenho em movimento.

## Validação final na Raspberry — 40 segundos

Arquivo: `run/validacao_pi_estavel_20261113.json`.

| Medida | Resultado |
| --- | --- |
| Nuvens recebidas do MID360 | 399; 10,00 Hz |
| Mensagens da IMU | 7.998; 200,01 Hz |
| Odometria FAST-LIO | 400; 10,00 Hz |
| Nuvens registradas | 400; 10,00 Hz |
| Maior intervalo entre timestamps da IMU | 6,62 ms |
| Maior intervalo entre timestamps das nuvens registradas | 105,86 ms |
| Variação de posição X/Y/Z | 2,78 / 2,94 / 4,12 mm |
| Pontos por nuvem registrada, mínimo | 1.507 |
| Campos inválidos/posição não finita detectados | nenhum |

## Recepção no PC — 30 segundos

Arquivo: `run/validacao_pc_host_20261113.json`.

- 272 mensagens de odometria e 271 nuvens registradas após descoberta ROS.
- Frequência de ambas: aproximadamente 10 Hz.
- Variação de posição X/Y/Z: 2,47 / 3,49 / 4,80 mm.
- Nenhum erro de consistência detectado. Nuvem bruta e IMU não são transmitidas
  ao PC neste fluxo; seus contadores são zero nessa validação de saídas.

O RViz foi inspecionado visualmente: nuvem presente, `Global Status: Ok`,
ferramenta `Move Camera`, vista `Orbit`, janela ampliada e aproximadamente
20 FPS. Captura: `run/rviz_final_20261113.png`.

## Verificações auxiliares

- Compilação ARM64 do adaptador concluída na Raspberry.
- Sintaxe dos scripts shell e compilação dos arquivos Python verificadas.
- Mapa visual: redução por voxel, limite de capacidade e rejeição de frame
  incorreto exercitados com pontos sintéticos.
- O receiver publica TF a partir da própria odometria recebida; não recalcula
  a localização. Validação de recepção do mapa/TF registrada em
  `run/validacao_visual_20261113.json`.
  Foram recebidos sete mapas em dez segundos (incluindo descoberta ROS),
  5.173 pontos no último mapa e TF `camera_init` → `body`, com resultado aprovado.
- Reiniciada somente a visualização: FAST-LIO continuou executando na Pi.
- Amostra de consumo da Pi após estabilização: container FAST-LIO cerca de
  193 MiB e 42% de um núcleo; adaptador cerca de 13 MiB; driver cerca de 46 MiB.
  São amostras, não limites ou garantias de utilização.

## Falhas encontradas antes da configuração final

Os primeiros ensaios não receberam dados entre os containers, apesar de o
driver funcionar dentro do próprio container. Apenas compartilhar o namespace
de PID não resolveu. A tentativa de compartilhar o IPC privado do driver foi
recusada pelo Docker porque esse namespace não era compartilhável.

A configuração final usa rede, PID e IPC do host nos três containers deste
teste físico. Após recriá-los assim, os dados passaram entre driver, adaptador
e estimador. O primeiro teste após a recriação incluiu lacunas de timestamps
na entrada durante a inicialização. Por isso foi realizado o ensaio estável
de 40 segundos registrado acima, com intervalos normais de nuvem e IMU.

Os ensaios sem dados permanecem nos logs `validacao_pi_20261113.txt`,
`validacao_pi_pid_20261113.txt` e `validacao_pi_ipc_20261113.txt`; não foram
contabilizados como testes aprovados.

Não foram iniciados comandos de movimento, missões ou rotinas de docking.
