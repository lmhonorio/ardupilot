# Cadastro único das estações de recarga

As entradas por QR e LiDAR leem, por padrão,
[`Rover/estacoes_recarga.txt`](../../estacoes_recarga.txt). As saídas também usam
esse cadastro, para manter a mesma referência de estação nos dois sentidos.
Os antigos `estacoes.json` e `estacoes_lidar.json` ficam como referência histórica;
não são mais os arquivos de configuração das rotinas.

O arquivo contém todas as estações conhecidas do cenário. Atualmente há uma:
`container_gravel_plane`. Acrescente uma linha para cada nova estação real do
mundo; cadastrar uma linha não cria um modelo no Gazebo.

## Formato semelhante ao locations.txt

```text
NOME=lat_4m,lon_4m,lat_2m,lon_2m,QR,comprimento_m,largura_m,altura_m,alvo_ate_qr_m
```

| Campo | Significado |
| --- | --- |
| `NOME` | Nome único, usando letras, números, `_` ou `-` |
| `lat_4m`, `lon_4m` | GPS do ponto externo a **4 m da abertura**, sobre o eixo central |
| `lat_2m`, `lon_2m` | GPS do ponto externo a **2 m da abertura**, sobre o mesmo eixo |
| `QR` | Conteúdo exato do QR dentro da estação; deve ser único no cadastro |
| `comprimento_m` | Vão livre interno, da abertura ao fundo |
| `largura_m`, `altura_m` | Vão livre interno nas outras duas dimensões |
| `alvo_ate_qr_m` | Distância longitudinal do centro desejado de parada ao plano do QR |

As coordenadas são latitude/longitude em graus decimais; as dimensões estão em
metros. Use **ponto decimal** e coordenadas com pelo menos oito casas decimais.
Linhas vazias são ignoradas e comentários ocupam linhas iniciadas por `#`.
Não use comentários depois dos dados. Se o QR contiver vírgulas, coloque o
conteúdo entre aspas duplas, conforme CSV; uma aspa interna vira `""`.

## Sentido e interpretação do eixo

```text
P4 -------- 2 m -------- P2 -------- 2 m -------- ABERTURA ---- CENTRO ---- QR
                  sentido de entrada ------------------------------->
```

P4 e P2 ficam **fora** do contêiner. A ordem P4 → P2 determina o rumo de entrada
(norte = 0°, leste = 90°). Inverter os pontos inverte o sentido calculado; o
programa não tem como saber que os pontos foram medidos do lado errado.

A abertura é calculada prolongando o eixo por 2 m depois de P2. O centro fica
mais `comprimento_m / 2` para dentro. Para um contêiner de 1 m, portanto:

- P4 está a 4,5 m do centro, não a 4 m.
- P2 está a 2,5 m do centro, não a 2 m.
- O rumo é calculado pelos pontos; não há outro campo de rumo para editar.

O leitor rejeita arquivo vazio, nomes/QR duplicados, coordenadas inválidas,
dimensões não positivas e pontos cuja separação difira de 2 m por mais de
0,10 m. Essa checagem valida os dados; confira também a posição física da
abertura no cenário.

## Estação atual — gravel_plane / SEParnaiba

```text
container_gravel_plane=-3.123167965999,-41.764490501318,-3.123167966000,-41.764472508288,ROVER_DOCKING_STATION_01,1.0,1.0,1.0,0.499
```

| Referência | Posição no mundo Gazebo |
| --- | --- |
| P4 | x = 0,5 m; y = −2 m |
| P2 | x = 2,5 m; y = −2 m |
| Abertura | x = 4,5 m; y = −2 m |
| Centro de parada | x = 5 m; y = −2 m |

O rumo é aproximadamente 90°. O centro GPS calculado mantém a referência
anterior: latitude −3,123167966 e longitude −41,764450017. O QR está a 0,499 m
do centro, conforme o modelo atual.

## Conferir o arquivo antes de iniciar

Este comando apenas lê e valida o cadastro, sem conectar ao rover:

```bash
cd ~/ardupilot/Rover
source ros2/setup.bash
python python/common/docking_stations.py
```

Ele imprime nome, rumo calculado, GPS do centro e conteúdo do QR de cada estação.
Para outro arquivo:

```bash
python python/common/docking_stations.py --arquivo /caminho/minhas_estacoes.txt
```

## Acionar a entrada

Por QR, selecionando automaticamente a estação cadastrada mais próxima:

```bash
cd ~/ardupilot/Rover
./ros2/service.sh docking-qr
```

Para restringir a ativação a uma estação pelo nome:

```bash
./ros2/service.sh docking-qr --estacao container_gravel_plane
```

Por LiDAR, com FAST-LIO e mapa local já funcionando:

```bash
./ros2/service.sh docking-lidar
```

Ou selecionando explicitamente:

```bash
./ros2/service.sh docking-lidar --estacao container_gravel_plane
```

Ambas as entradas usam o centro calculado para selecionar a estação e verificar
a distância de ativação de menos de 8 m, com rover armado em GUIDED. O QR usa
o código cadastrado, o rumo do eixo como referência inicial e a distância ao QR
para reconstruir o alvo visual. O LiDAR usa o centro, o rumo e as dimensões
internas para associar a geometria medida à estação.

**Os dois pontos definem o eixo; não são enviados como itens de missão.** O
monitor continua sem criar ou alterar missão. A aproximação e a entrada finais
continuam sendo controladas pela percepção QR ou LiDAR, conforme a rotina.

Todos os comandos aceitam outro arquivo por `--estacoes`:

```bash
./ros2/service.sh docking-qr --estacoes /caminho/minhas_estacoes.txt
./ros2/service.sh docking-lidar --estacoes /caminho/minhas_estacoes.txt
```

Execute apenas um controlador de docking por vez. Após editar o TXT, encerre
o monitor com Ctrl+C e abra-o novamente: o cadastro é carregado ao iniciar.

Na entrada imediata por QR (`--uma-vez`), use `--estacao NOME` se o cadastro
tiver várias estações. Na saída, escolha a estação onde o rover está:

```bash
./ros2/service.sh sair-qr --estacao container_gravel_plane
```

O fluxo completo de armamento, troca de modo e saída está no
[tutorial de entrada e saída](TUTORIAL_ENTRADA_SAIDA_ROS2.md). Os ensaios em
movimento da revisão anterior permanecem no
[relatório de validação](VALIDACAO_ALINHAMENTO_VELOCIDADE_ROS2.md); a mudança de
cadastro foi verificada separadamente por testes de leitura, geometria e
integração, sem representar novos ensaios de docking em movimento.
