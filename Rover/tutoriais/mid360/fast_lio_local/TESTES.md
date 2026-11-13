> Código deste componente: `Rover/mid360/fast_lio_local/`. Execute os comandos a partir de `~/ardupilot/Rover`; caminhos `run/` e `ws/` deste documento se referem ao diretório do código.

# Validação do ensaio local — 12/11/2026

FAST-LIO compilado e executado neste computador, MID360 físico adquirido
na Raspberry, domínio ROS 22 exclusivo. Os serviços da simulação e do
controle não foram iniciados nem modificados por esse teste.

## Correções durante a preparação

1. A compilação inicial foi interrompida antes de concluir. Após o reinício
   do computador, foi retomada e concluída em diretório próprio.
2. O teste de entrada identificou timestamps **absolutos em nanossegundos**
   no PointCloud2 nativo. O adaptador foi corrigido para subtrair o início
   da nuvem; não usa o modelo de tempo angular do parser genérico.
3. A primeira execução publicava mensagens, mas a posição divergiu. Seu
   cubo local de 60 m era incompatível com alcance de 40 m no algoritmo de
   remoção do mapa. Essa execução foi descartada. O arquivo `validation1.json`
   contém `passed=true` apenas porque a verificação então conferia transporte
   e saídas finitas; isso **não significa localização correta**.
4. A configuração final usa cubo 200 m, alcance 40 m e um bloqueio no
   iniciador para recusar cubos menores ou iguais a três vezes o alcance.
   Foi adicionada a verificação opcional de estabilidade `--stationary`.

## Execução final

Registro: `run/20261112_142704/`.
Validação: `run/validation2.json`, janela de 40 s.

| Observação | Resultado |
| --- | --- |
| Nuvens de entrada | 368; aproximadamente 9,97 Hz após descoberta |
| IMU observada | 7.381; aproximadamente 198,76 Hz após descoberta |
| Odometria e nuvem registrada | 374 mensagens cada; aproximadamente 10 Hz |
| Tempos relativos por ponto | 0 até aproximadamente 100,70 ms |
| Menor nuvem registrada | 1.550 pontos após filtragem |
| Variação da posição estimada X/Y/Z | 2,82 / 2,75 / 4,85 mm |
| Posição final relativa à inicialização | −0,00070 / −0,00546 / 0,02168 m |
| Erros de layout/valores não finitos | Nenhum observado |
| RViz | Nuvem alinhada visível e Global Status OK |

A variação pequena é estabilidade observada nessa janela, **não precisão
absoluta comprovada por referência externa**. O operador não confirmou por
mensagem se houve movimento; nenhuma trajetória física foi comandada.
Não houve validação de percurso, retorno ao ponto inicial ou fechamento de laços.

O maior intervalo de timestamp observado foi 0,2006 s na nuvem e 0,2344 s
na IMU. Portanto, este ensaio não comprova ausência de lacunas no fluxo
sensor/driver/DDS. O transporte SSH não apresentou quebra de sequência
entre mensagens recebidas pela fonte e publicadas no destino; isso não
prova que cada amostra produzida pelo hardware tenha sido recebida.

Após aproximadamente 119 s, o transporte registrava 1.158 nuvens e 23.168
mensagens IMU sem quebra de sequência. Uma amostra do processo FAST-LIO
indicou 74,4% de CPU (100% equivale a um núcleo) e 1,1% da RAM do computador.
Esses números não representam máximos nem um teste prolongado de carga.

## Evidências

- `run/build-final2.log`: compilação concluída dos dois pacotes;
- `run/probe_inputs.json`: identificação inicial do formato de tempo;
- `run/validation1.json`: primeira execução descartada por divergência;
- `run/validation2.json`: verificação final, inclusive critério de variação;
- `run/20261112_142704/`: logs de FAST-LIO, transporte, adaptador e RViz;
- `run/rviz_final.png`: captura da visualização local.

A pasta `run/` é ignorada pelo Git. O teste foi deixado em execução para
observação. Use `./mid360/fast_lio_local/parar.sh` para encerrar apenas esse
ensaio. Extrínsecos e qualidade em movimento ainda precisam ser validados
antes de qualquer integração com o rover real.

## Recuperação em 13/11/2026

A execução `20261113_100248` encerrou com `EOFError: Fonte do teste encerrou`.
Na inspeção, o driver permanecia aberto havia aproximadamente 20 horas, mas
`eth0` já não possuía `192.168.1.50/24`, endereço de destino dos dados do sensor.
Não foi determinado o evento que removeu o endereço temporário.
A fonte parou por falta de progresso e o supervisor encerrou os demais
processos; não foi um encerramento causado por manter o sensor parado.

O iniciador passou a verificar/restaurar esse endereço antes de abrir o teste,
reiniciando somente o driver físico quando a restauração for necessária.
Também verifica ping ao MID360 e registra explicitamente falta de dados na
fonte. A recuperação abriu a execução `20261113_100419`, inicializou a IMU
e o mapa novamente. Evidências em `run/recovery_20261113.log` e
`run/recovery_validation_20261113.json`.
