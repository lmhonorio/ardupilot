# Validação da implantação na Raspberry

Destino: `leo@192.168.0.198`, Raspberry Pi 4 B, ARM64, 4 GB de RAM,
Ubuntu 24.04. Data: 11/11/2026.

**Ciclo completo aprovado:** saída de ré, missão `Missions/new_argo.plan` com
seis waypoints, retorno, docking LiDAR, desarmamento e recarga simulada a 100%.
Gazebo/SITL/RViz e o cliente de missão ficaram no computador; SLAM, geometria,
gerenciador e distribuição MAVLink executaram na Raspberry. Perfil sem QR e
sem desvio, com fator de tempo real 0,5. Os ensaios anteriores e suas falhas
estão registrados abaixo; eles não contam como ciclos aprovados.

## Resultado do ciclo completo

O sétimo ensaio começou com o rover estacionado e carregado. A chamada de
missão partiu deste computador. A saída confirmou 2,965 m de recuo por geometria
LiDAR, 2,940 m pela odometria FAST-LIO e cerca de 2,3 cm de desvio lateral.
AUTO só começou após a confirmação de parada fora da estação.

Depois dos seis waypoints, o gerenciador entrou no container e terminou em
`RECARGA_CONCLUIDA`, HOLD, desarmado, com `BATTERY_STATUS` do próprio SITL
confirmando 100%. Duração da solicitação até a conclusão: 381,85 s reais.

| Erro final, referência independente do Gazebo | Medido |
| --- | ---: |
| Longitudinal, em relação ao centro | -0,00174 m |
| Lateral | +0,00661 m |
| Rumo | +0,527° |

Esses valores pertencem a um ensaio em simulação; não são uma especificação
de precisão do veículo físico. [Resultado e fases](../../python/run/companion_validation/20261111/cycle7/result.json).
Imagem desse ensaio: `sha256:8e9c1499820efa5e250dea27f86129579ed8f3f6038720a1761436355e382e49`.

## Verificações de software e comunicação

- Docker Engine e Compose instalados pelo repositório oficial.
- 229 testes de software passaram no computador de desenvolvimento, incluindo
  oito testes de isolamento dos perfis: serial recusada em simulação, configuração
  física não validada recusada, recarga artificial ausente no perfil real,
  calibração inválida recusada, e supervisão física sem dependência de SIMSTATE.
- Três testes adicionais verificaram a limitação de `/clock`: timestamps exatos,
  pausa e reinício do tempo simulado, e rejeição de taxas inválidas.
- Três testes verificaram retransmissão de `MISSION_REQUEST_LIST` após perda
  de resposta, falha limitada em caso de perda total e missão vazia legítima.
- Dois testes adicionais verificaram que posição apenas prevista/constante do
  EKF não autoriza navegação e que a falta dessa confirmação impede armamento/saída.
- Sintaxe dos scripts Bash e compilação sintática dos módulos Python verificadas.
- Descoberta DDS entre os dois computadores confirmada, sem configurar IP fixo.
- Amostra inicial de 12 s na Raspberry: 703 mensagens IMU, 89 nuvens e 783
  mensagens de relógio; idade ROS das nuvens de 0,004 a 0,112 s.
- A nuvem de origem contém 46.080 pontos por mensagem. A configuração ARM64
  amostra a entrada do estimador para reduzir custo de conversão/processamento;
  o ajuste das paredes continua usando a nuvem original.

Isoladamente, a amostra de rede e os testes de software não confirmam desempenho
do SLAM, controle ou docking na Raspberry. Esses resultados precisam ser separados
dos ensaios do container em funcionamento e de qualquer teste do veículo real.

## Imagem ARM64 e ensaios de transporte

A imagem foi construída nativamente na Raspberry: driver Livox/SDK2, FAST-LIO,
interfaces de missão e ponte C++ de nuvem. O teste executado dentro da imagem
confirmou bibliotecas nativas, importação dos controladores, tipos ROS, pymavlink,
CvBridge e codificação/decodificação de um QR conhecido.
[Log do teste da imagem](../../python/run/companion_validation/20261111/image-smoke-final.log).

Problemas observados e corrigidos durante a integração:

| Verificação | Resultado e alteração |
| --- | --- |
| Relógio compartilhando a ponte com nuvens/imagens | Atrasos de segundos. Processos separados; relógio exato limitado a 50 Hz de simulação. |
| Recepção UDP na Raspberry | Contador de descartes crescia com buffers de 212 KiB; buffers aumentados para 8 MiB. |
| Transporte interno de uma nuvem de 1,4 MiB, dois processos isolados da rede externa | Configuração SHM padrão: 10/20 mensagens intactas. Segmentos de 16 MiB: 20/20. |
| Carga total no primeiro perfil em tempo real | Próxima de 400% de CPU, temperatura próxima de 85 °C. Perfil da Pi 4 usa simulação a 0,5 do tempo real, amostragem no adaptador FAST-LIO e repasse nativo da nuvem completa. |

Os testes de transporte conferem o checksum de todo o payload, número de pontos,
frame e sequência. [Padrão](../../python/run/companion_validation/20261111/shm-default-test.log)
e [16 MiB](../../python/run/companion_validation/20261111/shm-16mb-test.log).

## Ciclos de missão pela Raspberry

O primeiro ciclo enviou `Missions/new_argo.plan` deste computador pelo serviço ROS.
O gerenciador na Raspberry executou os waypoints e acionou o docking automaticamente.
A entrada foi interrompida por `Varredura de protecao desatualizada`, com retorno
a HOLD. Esse ensaio ocorreu antes de corrigir o tamanho dos segmentos SHM;
**não é um ciclo aprovado de docking/recarga**.
[Resultado do primeiro ciclo](../../python/run/companion_validation/20261111/cycle1/result.json).

O segundo ciclo, já com SHM de 16 MiB, iniciou a missão, mas parou em HOLD após
perda da resposta `MISSION_COUNT`. A solicitação de quantidade de itens tinha
uma única tentativa. Foi corrigida para até três tentativas dentro do limite
de tempo existente, como já ocorria na leitura dos itens individuais.
[Resultado do segundo ciclo](../../python/run/companion_validation/20261111/cycle2/result.json).

O terceiro ciclo completou a missão, alinhou e entrou parcialmente no container,
mas interrompeu por idade excessiva da pose LiDAR (1,128 s de simulação),
retornando a HOLD. Naquele ensaio, as pontes do computador ainda usavam sockets
com os buffers antigos; o operador aplicou posteriormente o ajuste de 8 MiB
também neste computador. [Resultado](../../python/run/companion_validation/20261111/cycle3/result.json).

O ajuste geométrico isolado na Raspberry, com a mesma nuvem capturada e limite
de 1.200 pontos, levou 0,156–0,172 s. Comparado à pose independente do Gazebo,
nesse quadro estacionário o erro foi inferior a 1 mm em frente/lateral e a
0,1° no rumo. Amostrar 600 ou 300 pontos trouxe pouco ganho de tempo; o limite
original de 1.200 foi mantido. Esse ensaio de um quadro não prova precisão dinâmica.
[Medições](../../python/run/companion_validation/20261111/fit_benchmark.json)
e [referência do Gazebo](../../python/run/companion_validation/20261111/fit_truth.json).

O quarto ensaio tentou iniciar logo após reiniciar o SITL. AUTO foi recusado:
os registros mostram `EKF_STATUS_REPORT.flags=167`, com posição constante e sem
posição horizontal absoluta. Foi acrescentada uma espera limitada por EKF válido
**antes** do armamento/saída. A verificação do próprio ArduPilot permanece ativa.
[Resultado](../../python/run/companion_validation/20261111/cycle4/result.json).

O quinto ensaio, com buffers novos nos dois computadores, completou a missão
curta de dois pontos e entrou parcialmente, mas atingiu o limite de tempo da
fase de entrada. O processo de adaptação Python consumia cerca de 71% de um
núcleo, e o conjunto continuava próximo da capacidade da Pi 4. A conversão
simulada foi transferida para C++, preservando os pontos selecionados e seus
tempos; a publicação do mapa passou a usar arrays NumPy diretamente.
Os limites de validade e os controladores não foram relaxados.
[Resultado](../../python/run/companion_validation/20261111/cycle5/result.json).

O teste do adaptador C++, executado na imagem ARM64, aprovou amostragem 1, 8 e
16 com 1.017, 128 e 64 pontos de saída. Verificou igualdade de XYZ e ordem,
campos fora da ordem usual, preenchimento entre linhas de uma nuvem organizada,
filtragem de NaN/Inf/alcance e timestamps de aquisição exatos.
[Log](../../python/run/companion_validation/20261111/native-adapter-test.log).

No sexto ciclo, com adaptação nativa, a missão e a entrada avançaram até o
centro. A referência independente do Gazebo mediu erro longitudinal de
aproximadamente -2,9 cm, lateral de +0,07 cm e rumo de +0,73° após parar.
Ainda assim, o ensaio **falhou**: a confirmação final não conseguiu reunir
observações LiDAR recentes dentro do prazo. Retornou a HOLD; o desarmamento
foi solicitado pelo teste após a falha. Não houve recarga automática aprovada.
[Resultado](../../python/run/companion_validation/20261111/cycle6/result.json)
e [pose independente](../../python/run/companion_validation/20261111/cycle6/truth.json).

A instrumentação identificou nuvem recente na entrada (idade ROS 0,068 s),
mas ajuste e preparação geométrica consumindo 1,162 + 0,290 s reais no processo
do gerenciador. O cálculo foi separado em um trabalhador criado por `spawn`,
sem conexões ROS/MAVLink. O algoritmo geométrico e os limites de validade são
os mesmos. Um teste adicional confirmou pose, varredura e rejeição de parede
isolada nesse processo.

Após corrigir SHM, uma amostra de 12 segundos com todos os serviços recebeu
48 nuvens completas, 51 mensagens de odometria e 52 nuvens locais, sem nuvens
vazias. Idade ROS média/máxima: nuvem atual 0,095/0,208 s; odometria
0,168/0,308 s; nuvem local 0,187/0,308 s. São tempos de simulação, com fator
0,5; o diagnóstico adiciona carga temporária à Raspberry.
[Amostra](../../python/run/companion_validation/20261111/network-final-half.json).

O teste de saída com observações repetidas reproduziu pulsos de parada entre
varreduras ainda válidas. A correção mantém o comando entre essas amostras;
expiração, perda de dados ou regressão do tempo continuam interrompendo o movimento.
[Reprodução antes da correção](../../python/run/companion_validation/20261111/exit-regression-before.log)
e [regressões após a correção](../../python/run/companion_validation/20261111/regression-final.log).

A preparação dos voxels foi otimizada mantendo igualdade exata dos pontos e
da ordem de saída. Na Raspberry, para 31.374 pontos, a identificação dos
voxels passou de 0,10–0,31 s para 0,02–0,04 s; quatro repetições em cada versão.
O ajuste e a proteção agora reutilizam a mesma varredura preparada.
[Benchmark](../../python/run/companion_validation/20261111/voxel_benchmark.json).

## Imagem final e mapa acumulado

Depois do ciclo aprovado, a última alteração funcional tratou a retenção do
mapa visual: uma nuvem atrasada invalida a geometria local usada na proteção,
mas preserva o mapa acumulado. Regressão do relógio e descontinuidade da
odometria continuam descartando as referências incompatíveis.

O teste de integração ROS confirmou esses comportamentos no computador e na
imagem ARM64. Não foi repetida uma missão completa após essa alteração;
foram executados os testes específicos do mapa, as 229 regressões de software
e o teste de dependências da imagem.
[Teste local](../../python/run/companion_validation/20261111/map-retention.log)
e [teste ARM64](../../python/run/companion_validation/20261111/map-retention-arm64.log).

Imagem final instalada:
`sha256:a6840543dd1f80de1f2a2bde91bfa77143d3bf35d29ff2a9b1f49806215ec145`.
Após recriar o container, a reconciliação confirmou `RECARGA_CONCLUIDA`,
`inside=true`, HOLD, desarmado, bateria SITL em 100% e nenhuma falha ativa.
O container está `healthy`. Os serviços permaneceram abertos.
[Status recebido no computador](../../python/run/companion_validation/20261111/pc-status-final.txt),
[metadados da imagem](../../python/run/companion_validation/20261111/runtime-final.txt)
e [manifesto dos fontes](../../python/run/companion_validation/20261111/source_manifest.json).

## Limites do perfil físico

Não foi conectado ou acionado um rover físico. A configuração real permanece
bloqueada até comissionamento. Firmware com `DOCK_LOCK`, calibração dos sensores,
envelope, coordenadas das estações, dispositivos e integração com o carregador
real são requisitos próprios; não são comprovados por testes em Gazebo.


Amostra adicional antes da migração (10 s, Raspberry consumindo a rede):
75 nuvens, 772 mensagens IMU, idade média da nuvem 0,025 s e máxima 0,060 s.
A odometria ainda calculada no computador teve idade média de 0,612 s nessa
amostra. Esse valor descreve a configuração anterior; não é uma aprovação
do desempenho ARM64. [Dados brutos](../../python/run/companion_validation/20261111/network_pi_before.json).

