> Código deste componente: `Rover/mid360/`. Execute os comandos a partir de `~/ardupilot/Rover`; caminhos `run/` e `ws/` deste documento se referem ao diretório do código.

# MID360 físico — resultados de 12/11/2026

Teste independente, sem habilitar navegação, docking ou SLAM físico.
Nenhum arquivo da simulação foi alterado nesta tarefa. A correção inicialmente
feita no modelo YAML do companion foi desfeita; o YAML correto deste teste
está somente em `mid360/livox.yaml`.

## Identificação e ligação

- Serial informado pelo operador: `47MCNAH0037261`.
- ARP observado: IP `192.168.1.161`, MAC `0c:9a:e6:cf:42:1a`.
- O IP corresponde ao sufixo 61 do serial; não foi feita leitura independente
  do serial pelo SDK.
- Raspberry: SSH `192.168.0.167`, Ethernet `192.168.0.166`.
- Endereço secundário de aquisição: `192.168.1.50/24`, temporário em `eth0`.
- Ping inicial Pi → sensor: 3/3 respostas, 0% de perda; média 2,393 ms.
- Driver Livox ROS2 1.2.8 da imagem ARM64 já instalada: inicializou,
  reconheceu o sensor configurado como tipo 8 e confirmou IMU habilitada.

Não foram alterados IP do sensor, roteador, rota padrão ou endereços de SSH.
O driver configura o destino UDP para o endereço secundário e usa scan normal.

## Configuração entregue

Driver físico em `rover-mid360`, domínio ROS 21 restrito ao localhost na Pi.
Saída nativa PointCloud2 (`xfer_format: 0`), aproximadamente 20 mil pontos
por mensagem. Não há conversor Python de nuvens na Pi.

A ponte SSH transmite mensagens ROS serializadas com compressão sem perda;
o computador republica e exibe no RViz, também no domínio 21 local. A fila
de envio é limitada e pode descartar mensagens antigas em congestionamento.
Portanto, é transporte para este teste/visualização, não alimentação do SLAM.
Na integração futura, FAST-LIO fica no rover, consumindo os dados locais completos.

## Medições finais

| Verificação | Resultado |
| --- | --- |
| Recepção na Pi, janela de 20 s | 199 nuvens; taxa entre primeira/última: 9,95 Hz |
| IMU na Pi | 4.001 mensagens; aproximadamente 200 Hz |
| Pontos por nuvem | 19.872–20.064 |
| Inspeção XYZ na Pi | 3.979.872 pontos finitos; 2.198.748 retornos com distância maior que 10 cm |
| Campos nativos | x, y, z, intensity, tag, line, timestamp |
| Layout e timestamps na Pi | Sem erros de layout; timestamps crescentes; intervalo máximo observado de 0,200 s |
| Recepção no computador via SSH, janela de 30 s | 262 nuvens; taxa entre primeira/última: 9,72 Hz |
| IMU recebida no computador | 3.735 mensagens; taxa entre primeira/última: 137,54 Hz |
| Inspeção no computador | 5.239.200 pontos finitos; layout válido; timestamps crescentes |
| RViz no computador | Nuvem física visível; janela maximizada; `livox_frame` |

As taxas entre primeira/última mensagem não incluem a espera inicial de
descoberta; não equivalem a dividir o total por toda a janela. A taxa menor
da IMU no computador reflete o descarte permitido pela fila da ponte, não
a taxa de aquisição na Pi. O teste de recepção não exige perda zero e não
certifica precisão, sincronismo para navegação ou desempenho prolongado.

A aceleração do driver parado tem norma próxima de 1. Isso foi registrado
como valor bruto; a convenção de unidades deve ser conferida ao integrar o
estimador. Nenhuma escala/extrínseco foi calibrado neste ensaio.

## Tentativas anteriores e decisão de transporte

O primeiro ensaio usava CustomMsg e conversor Python separado. A inspeção
Python de nuvens grandes interferia na contagem da IMU no mesmo executor.
Medições separadas confirmaram 200 Hz na aquisição da IMU. Esse conversor foi
removido da solução final, usando a saída PointCloud2 nativa do driver.

DDS direto Pi → Wi-Fi do computador apresentou perda/intermitência de nuvens.
Foram testados Best Effort, Reliable e fragmentos DDS menores. Houve janelas
sem nuvem ou sem dados recentes no fim do teste, mesmo recebendo alguns pacotes.
O perfil experimental de transporte foi removido. A ponte SSH passou na
verificação final e mostrou a nuvem na tela. Nenhum desses ajustes foi aplicado
ao transporte dos serviços da simulação.

## Evidências e estado deixado

Arquivos em `mid360/run/` (ignorados pelo Git):

- `teste_pi_isolado.json`: medição final na Raspberry;
- `teste_pc_ssh.json`: medição final no computador;
- `driver_final.log`, `bridge.log`, `rviz.log`: registros;
- `rviz_final.png`: captura da janela mostrando a nuvem real;
- demais `teste_*.json`/`.log`: tentativas anteriores, inclusive falhas.

Driver, ponte e RViz foram deixados abertos para o operador acompanhar.
Os PIDs locais estão em `run/bridge.pid` e `run/rviz.pid`, válidos somente
enquanto esses processos existirem. Para encerrar, feche o RViz e execute
`./mid360/control.sh down`; a ponte encerra quando o driver desaparece.
Para remover também o endereço temporário, use `network-down`.
