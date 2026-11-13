> Código deste componente: `Rover/mid360/`. Execute os comandos a partir de `~/ardupilot/Rover`; caminhos `run/` e `ws/` deste documento se referem ao diretório do código.

# MID360 físico — conexão e teste pela Raspberry

Escopo atual: **aquisição, teste FAST-LIO e visualização independentes**. Nenhum arquivo de
simulação é modificado por esta rotina. Os containers desta pasta usam as
bibliotecas da imagem já instalada, mas não iniciam sua pilha de gerenciamento.
Não há integração com Gazebo, SITL, missão, docking ou controle do rover.

Sensor informado: **47MCNAH0037261**. IP encontrado: **192.168.1.161**.
MAC observado: `0c:9a:e6:cf:42:1a`. Raspberry de aquisição:
`rover@192.168.0.167` (SSH), Ethernet `192.168.0.166`.

O [guia oficial Livox](https://dl.djicdn.com/downloads/Livox/Mid-360/QSG/Livox_Mid-360_Quick_Start_Guide_multi.pdf)
descreve IP de fábrica `192.168.1.1XX`, onde XX são os dois últimos dígitos
do número de série. Neste sensor, `61` corresponde a `192.168.1.161`.
Como esse endereço é estático, não espere encontrá-lo como concessão DHCP
do roteador. O cabo de rede não substitui a alimentação externa do sensor.

## Ligação usada

- MID360 alimentado externamente, cabo Ethernet em uma porta LAN do roteador.
- Raspberry ligada por Ethernet à mesma LAN, sem isolamento entre portas.
- Computador conectado ao mesmo roteador por Wi-Fi.

Foi adicionado **somente** `192.168.1.50/24` à interface `eth0` da Pi, como
endereço secundário temporário. IPs `192.168.0.166`/`.167`, rota padrão e SSH
permanecem. O IP do sensor e a configuração do roteador não foram alterados.
O driver configura o destino de dados do sensor para `192.168.1.50`.

O endereço secundário desaparece ao reiniciar a Pi. O comando `up` abaixo o
recria, após verificar conflito ARP. Não é necessário configurar
`192.168.1.x` neste computador: ele recebe os dados da Pi por uma ponte SSH e os republica localmente para o RViz.

## Iniciar — comandos neste computador

Pré-requisito: imagem `rover-companion:jazzy` instalada na Pi, como no
[tutorial de instalação](../companion/INSTALACAO_PI5_SIMULACAO.md).

```bash
cd ~/ardupilot/Rover
./mid360/control.sh up
./mid360/control.sh status
./mid360/control.sh test
```

`up` envia os arquivos desta pasta para `~/mid360` na Pi e inicia somente o driver
no container `rover-mid360`. `test` mede nuvem e IMU por 20 segundos;
o resultado é JSON. O driver publica PointCloud2 nativo (`xfer_format: 0`),
sem nó conversor Python na Raspberry. O formato nativo contém XYZ, intensidade, tag, linha e timestamp.

O nome do nó no YAML deve ser `livox_driver_node`. Um nome diferente deixa
o driver com `Config file: path_default` e `Init lds lidar fail`.

Para acompanhar mensagens do driver:

```bash
./mid360/control.sh logs
```

Ctrl+C fecha apenas a visualização dos logs. Não abra Livox Viewer ou outro
driver controlando o mesmo sensor simultaneamente: eles podem trocar o
destino dos pacotes um do outro.

## Ver a nuvem no RViz deste computador

O teste físico usa **ROS_DOMAIN_ID=21**, descoberta restrita ao localhost e relógio real, independente do
domínio 0 do Gazebo/SITL e do domínio 20 do perfil do rover real.
Ele não inicia missão, docking, MAVProxy nem comandos ao autopiloto.

Terminal 1, neste computador: mantenha a ponte de dados aberta.

```bash
cd ~/ardupilot/Rover
./mid360/ponte_pc.sh
```

Terminal 2, neste computador:

```bash
cd ~/ardupilot/Rover
./mid360/visualizar.sh
```

A própria saída nativa do driver publica, localmente na Pi, `/mid360/points` como
`sensor_msgs/PointCloud2`, com frame `livox_frame`. RViz e eventual
processamento adicional rodam neste computador. Não há container conversor.
A ponte SSH transporta as mensagens serializadas com compressão sem perda;
não transforma coordenadas nem timestamps. `receber.py` republica no computador,
no mesmo domínio 21 local. A configuração RViz seleciona tópico, Fixed Frame
e QoS Best Effort. O acesso SSH precisa funcionar sem senha interativa
(chave já configurada nesta instalação).
O desenho representa a varredura corrente, **não um mapa SLAM acumulado**.
Sem TF/odometria, mover o sensor não registra as varreduras num referencial global.

A IMU fica em `/mid360/imu` (`sensor_msgs/Imu`). Para validar os dois fluxos
recebidos neste computador, sem abrir RViz:

```bash
./mid360/testar_pc.sh
```

Com a ponte em execução, o teste confere campos XYZ, tamanho dos pacotes, pontos finitos, retornos
não nulos, sequência temporal e recepção da IMU. Ele não calibra o sensor.
Os auxiliares usam domínio 21 e relógio real apenas em seus próprios processos;
não modificam o ambiente dos terminais da simulação.

## Encerrar

Feche o RViz e interrompa `ponte_pc.sh` com Ctrl+C. Depois:

```bash
cd ~/ardupilot/Rover
./mid360/control.sh down
```

Encerra somente o driver desta pasta; não encerra o container de
simulação `rover-companion`. Não desliga a fonte do MID360. Para também
remover o endereço secundário criado pelo script:

```bash
./mid360/control.sh network-down
```

Esse comando consulta `~/mid360/run/network.json` e só remove o endereço
registrado por esta rotina. Não remove os IPs de SSH ou a rota padrão.
Os serviços deste teste não têm reinício automático após ligar a Pi.
Se a ponte perder a conexão/sensor por 20 s, ela encerra em vez de continuar
publicando dados antigos; execute `ponte_pc.sh` novamente após recuperar o link.
Não é possível abrir duas cópias da ponte no mesmo computador: há um lock local.

## Diagnóstico e arquivos

O sensor deve responder por Ethernet na Pi:

```bash
ssh rover@192.168.0.167 'ip route get 192.168.1.161; ping -c 3 192.168.1.161'
```

Se não responder, confira alimentação, porta LAN, cabo, isolamento de rede
e endereço secundário. Não use a porta WAN do roteador para essa ligação.
Para repetir a descoberta sem alterar IPs, depois de `up` ter enviado os arquivos:

```bash
ssh rover@192.168.0.167 \
  'docker run --rm --network host -v "$HOME/mid360:/mid360:ro" rover-companion:jazzy python /mid360/descobrir.py --interface eth0 --scan 192.168.1.0/24 --source-ip 192.168.0.166 --seconds 20'
```

| Arquivo | Função |
| --- | --- |
| `mid360.json` | IP do sensor, destino e portas UDP Livox |
| `livox.yaml` | Parâmetros ROS do driver físico |
| `rede.py` | Endereço secundário e verificação de conflito ARP |
| `descobrir.py` | Observação de ARP/UDP e consulta ARP opcional |
| `testar.py` | Contagem de mensagens, pontos finitos e timestamps |
| `compose.yaml`, `control.sh` | Inicialização, diagnóstico e encerramento remoto |
| `mid360.rviz`, `visualizar.sh` | Visualização da nuvem física no computador |
| `testar_pc.sh` | Validação da recepção pelo computador |
| `enviar.py` | Transporte de mensagens ROS serializadas, sem converter pontos, pela Pi |
| `receber.py`, `ponte_pc.sh` | Recepção SSH e publicação local no computador |
| `run/` | Evidências locais de teste; não versionadas |

## Arquitetura atual e futura

Fluxo de aquisição bruta: MID360 → driver na Pi → ponte SSH → RViz no computador.
Nesse fluxo específico, a Pi não roda conversor de visualização, RViz ou SLAM.
O envio tem fila limitada e prefere a mensagem mais recente em congestionamento;
a recepção pelo computador não é uma gravação completa de todas as amostras.

O teste [FAST-LIO na Raspberry](fast_lio_raspberry/README.md) já executa
driver e estimador na Pi, consumindo as mensagens locais completas. O computador
fica com o RViz e o mapa visual acumulado. Esse teste usa sua própria ponte de
resultados. A ponte de aquisição bruta descrita acima não deve alimentar um
estimador: pode descartar amostras de IMU durante o envio. A integração com
controle, docking e navegação do rover real continua sendo uma etapa separada.

## Limites deste teste; integração futura

Estes testes comprovam comunicação e recepção, não calibração/extrínsecos,
sincronismo para navegação ou precisão de SLAM. No teste parado, a norma da
aceleração publicada pelo driver ficou próxima de 1; não interprete esse
valor automaticamente como m/s² nem mude a escala sem conferir a convenção
esperada pelo estimador. O driver desta imagem repassa os valores Livox.

A integração com o rover real fica para outra etapa, após definir calibração,
sincronismo e interfaces. Não copie estas configurações para a simulação.
Neste trabalho, os arquivos do companion e seus perfis foram preservados.
O teste de aquisição descrito acima não habilita SLAM, docking nem navegação
física. Resultados da aquisição: [TESTES.md](TESTES.md). O teste separado de
FAST-LIO e seus resultados estão em [fast_lio_raspberry](fast_lio_raspberry/README.md).

## FAST-LIO na Raspberry, visualização no PC

O teste independente com o processamento na Pi 5 está em
[fast_lio_raspberry/README.md](fast_lio_raspberry/README.md).
No PC, a partir de `~/ardupilot/Rover`:

```bash
./mid360/fast_lio_raspberry/iniciar.sh
```

Para encerrar somente este teste físico nas duas máquinas:

```bash
./mid360/fast_lio_raspberry/parar.sh
```

Não execute simultaneamente os testes `fast_lio_local` e
`fast_lio_raspberry`; escolha onde deseja processar o sensor.
