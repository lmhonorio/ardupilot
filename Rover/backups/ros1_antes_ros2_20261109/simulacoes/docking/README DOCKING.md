# Aproximação da estação e observação do QR

Para a alternativa sem câmera e sem QR, consulte [Entrada e saída com MID360](DOCKING_LIDAR.md).

> Para acionar o docking sem depender de missão, use a seção [Espera automática para docking](#espera-automática-para-docking-em-guided). O comando `volta_e_docking.py` abaixo carrega outra missão; não o execute junto com o monitor.

## Volta e entrada automática no contêiner

A missão [`volta_docking.waypoints`](volta_docking.waypoints), para o mundo `gravel_plane`, percorre aproximadamente os pontos `(-1,-2)`, `(-1,2)`, `(2,2)` e `(2,-2)` e termina em `(3.2,-2)` metros no referencial do Gazebo. A última reta aponta para a abertura do contêiner, em `x=4,5 m`. O arquivo pressupõe a origem SITL `SEParnaiba` em `-3.12315, -41.764495`; confira o mundo, a origem e os obstáculos antes de executar.

1. Inicie o Gazebo, o SITL e os demais componentes conforme o [passo a passo](../PASSO_A_PASSO_ROS_FAST_LIO.md), com a câmera Samsung habilitada e o rover fora do contêiner, parado e desarmado. Aguarde GPS, EKF e as verificações PreArm ficarem prontos.
2. Confirme que o MAVProxy está ativo, com saída `udp:127.0.0.1:14551` e `set mavfwd true`. O `sim_1_gzrover.sh` já configura essa saída. Abra o QGroundControl para acompanhar.
3. Em outro terminal, execute:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/ardupilot/Rover
/home/grin/.pyenv/versions/3.8.10/bin/python3.8 python/docking/volta_e_docking.py
```

O programa salva a missão anterior em `python/run/docking/volta_<data>/missao_anterior.json`, substitui a missão no veículo, arma e inicia AUTO. Após concluir a aproximação e estabilizar em HOLD, chama a entrada visual em GUIDED descrita abaixo. A entrada somente começa se o QR correto estiver visível e o alinhamento satisfizer os limites do controlador. Para interromper, use Ctrl+C no terminal ou `mode hold` no MAVProxy; acompanhe as mensagens e a restauração dos parâmetros de desvio.

Esta sequência completa ainda não foi validada em movimento: um ensaio isolado foi interrompido antes de armar porque o estado PreArm não ficou saudável. O arquivo de missão anterior é um registro JSON, não um arquivo WPL diretamente carregável. A entrada visual, isoladamente, foi ensaiada conforme [resultado e condições](RESULTADO_ENTRADA.md). Este programa é exclusivo para SITL; o estado PreArm e os sensores devem estar saudáveis, sem desabilitar verificações para forçar a execução.

A missão [`aproximacao_qr.waypoints`](aproximacao_qr.waypoints) foi gerada para `gravel_plane`, com origem GPS do SITL `SEParnaiba` em -3.12315, -41.764495. Os dois destinos no Gazebo são aproximadamente `(x=2, y=-2)` e `(x=3, y=-2)` m: o último fica 1,5 m antes da entrada, que está em x=4,5 m. O trecho final aponta para o fundo da estação, mas a trajetória, a orientação final e a precisão GPS **ainda não foram validadas**; raio de waypoint e desvio podem fazer o rover parar fora do alinhamento.

Para carregar a missão no MAVProxy após restaurar a simulação, use `wp load /home/grin/ardupilot/Rover/simulacoes/docking/aproximacao_qr.waypoints`. Isso substitui a missão no veículo; salve a anterior antes caso precise dela. `mode hold`, `wp set 0` e `mode auto` iniciam a missão se o rover estiver armado e as verificações de pré-armamento estiverem satisfeitas. Confirme no QGroundControl e acompanhe no Gazebo.

Para testar a leitura do QR pela câmera Samsung:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/ardupilot/Rover
/home/grin/.pyenv/versions/3.8.10/bin/python3.8 python/docking/observar_qr.py
```

O observador é somente leitura: identifica `ROVER_DOCKING_STATION_01`, apresenta distância aproximada e ângulo na imagem; não muda modo e não movimenta o rover. A estimativa assume que o QR está aproximadamente de frente e usa o tamanho efetivo do padrão impresso. Deve ser calibrada para ângulos e distâncias reais.

Uma câmera estática no Gazebo isolado leu o QR corrigido e estimou 2,49 m para uma distância geométrica de aproximadamente 2,50 m. Não foi testada a leitura pela câmera móvel do rover na posição de aproximação.

## Espera automática para docking em GUIDED

Após iniciar Gazebo e MAVProxy, deixe este programa aberto em um terminal separado:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/ardupilot/Rover
/home/grin/.pyenv/versions/3.8.10/bin/python3.8 python/docking/entrar_container.py
```

O programa escuta a saída local UDP 14551 do MAVProxy sem consultar, criar, enviar ou alterar missões. Ele tenta iniciar a entrada quando houver telemetria recente do SITL, rover armado em modo **GUIDED**, GPS 3D e distância horizontal **menor que 8 m do centro** de uma estação cadastrada. Não é necessário chegar a qualquer waypoint ou ter uma missão carregada.

As estações estão cadastradas em [`estacoes.json`](../../python/docking/estacoes.json), com nome, coordenadas, conteúdo do QR e distância calibrada da parede ao alvo visual. A estação inicial fica na latitude `-3.123167966`, longitude `-41.764450017`, calculada para `gravel_plane` e origem `SEParnaiba`. Altere ou acrescente cadastros para outros cenários e reinicie o monitor. O monitor seleciona a estação cadastrada mais próxima; o QR precisa corresponder ao dela. O GPS é aproximado.

Antes de mudar o modo, deixe o rover **parado**. Se o QR não estiver visível, o controlador busca a estação girando em sua direção pelo GPS cadastrado. O limite de 8 m apenas autoriza a tentativa; a detecção do QR e os demais limites ainda precisam ser satisfeitos. O controlador solicita HOLD durante as verificações e volta a GUIDED. Ele calcula um ponto de aproximação sobre o eixo do QR, busca esse ponto e alinha o rumo antes de entrar; ao entrar na região final, solicita HOLD imediatamente com margem de frenagem baseada na velocidade e só desarma depois de confirmar parada e verificar novamente a pose visual. Nessa região não faz novas manobras; se perder o QR, permanece em HOLD. Se uma tentativa falhar, leia o motivo; é preciso sair de GUIDED e ativá-lo novamente para outra tentativa. Depois de terminar, o programa volta a aguardar. Ctrl+C encerra a espera ou interrompe a entrada em andamento.

O monitor observa o **modo efetivo GUIDED** no heartbeat. Nesta versão do Rover, o item de missão `MAV_CMD_NAV_GUIDED_ENABLE` inicia um submodo guiado *dentro de AUTO* e não equivale necessariamente a uma troca do modo informado no heartbeat; ele, sozinho, não foi integrado a este gatilho. Nossa explicação anterior que tratava os dois como equivalentes estava incorreta. Uma troca efetiva de modo pode ser comandada pelo QGroundControl ou MAVProxy depois que o rover parar.

A ativação e o desarmamento foram testados com dados simulados nos testes de software; a sequência completa em movimento, inclusive o novo alinhamento e desarmamento, ainda não foi validada. As tolerâncias são de pose estimada pela câmera e não garantem precisão física. O processo é exclusivo para SITL e não deve compartilhar a porta 14551 com outro programa, inclusive `volta_e_docking.py`.

## Entrada visual imediata experimental

Depois que o rover estiver parado, com o QR da estação visível, execute em outro terminal:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/ardupilot/Rover
/home/grin/.pyenv/versions/3.8.10/bin/python3.8 python/docking/entrar_container.py --uma-vez
```

O programa é para este SITL e usa UDP 14551 via MAVProxy. Ele exige um rover armado, parado, em HOLD, AUTO ou GUIDED, telemetria SITL e câmera reconhecendo o QR correto. O controlador calcula a aproximação ao eixo a partir da pose estimada do QR; durante a entrada estreita usa limites mais restritos de alinhamento. O sistema comanda pan para zero e tilt inicial de -0,30 rad, confirma as juntas e ajusta o tilt conforme a aproximação para manter o QR visível. Se esses requisitos não forem satisfeitos, ele interrompe antes de comandar o movimento.

A posição relativa é calculada pela câmera e pelas dimensões do QR. O alvo nominal é o centro do contêiner, com compensação visual calibrada para 0,5335 m em relação ao QR; o programa transmite `LANDING_TARGET` em BODY_FRD ao ArduPilot pelo MAVProxy e envia setpoints de velocidade e giro em GUIDED. Esses dados não são fundidos automaticamente no EKF. A velocidade padrão é 0,08 m/s e está limitada a 0,12 m/s; use `--velocidade` para diminuir. O limite padrão de execução é 300 s, configurável com `--timeout` até 600 s. Ao iniciar, a rotina salva `OA_TYPE` e `AVOID_ENABLE` em `python/run/docking/<data>/parametros_anteriores.txt`. Ela solicita desabilitar o BendyRuler e a proximidade do desvio simples ao iniciar a entrada, mantendo o bit da cerca; a aplicação de `OA_TYPE` pode exigir reinício no ArduPilot, portanto confira o comportamento efetivo no SITL. Se o QR não for reconhecido, o controlador comanda avanço zero e gira em direção ao centro da estação usando o GPS e o rumo; após reencontrar o QR, retoma a aproximação visual. Se a imagem, GPS ou rumo não estiverem confiáveis, aguarda sem girar; a busca pode continuar sem limite de tempo. Ao terminar, detectar perda de telemetria ou alinhamento fora da tolerância, solicita HOLD e tenta restaurar os parâmetros salvos. Se houver falha na restauração, consulte esse arquivo e os erros impressos.

Ctrl+C solicita HOLD e restauração. Uma falha abrupta do computador, encerramento forçado ou perda prolongada da conexão pode impedir a restauração; confira `param show OA_TYPE` e `param show AVOID_ENABLE` antes de voltar à navegação. Não use em um veículo real.

O cálculo geométrico foi testado com imagens sintéticas e a troca de modo, transmissão e restauração foram verificadas em SITL isolado usando câmera simulada por imagem fixa. O comportamento anterior de perda da imagem causou HOLD no teste; a nova busca pelo GPS ainda não foi validada em movimento no Gazebo. Ensaios da entrada no Gazebo, com o rover inicialmente alinhado, terminaram com o rover dentro do contêiner; veja [resultado e condições](RESULTADO_ENTRADA.md). A aproximação completa desde um ponto arbitrário ainda não foi validada, e o QR pode sair do campo de visão, o pan/tilt e a geometria podem apresentar erros e a desaceleração depende dos controles em uso. Supervisione e mantenha acesso ao comando `mode hold` no MAVProxy. A condição de parada é a pose relativa estimada, não uma confirmação de colisão ou de contato de carregamento.

A nova aproximação foi verificada em modelo geométrico com a pose relatada (`forward=3,85`, `right=1,47`, `normal_yaw=-0,40`). Ela ainda não foi validada em movimento no Gazebo: a câmera pode perder o QR ao girar e o trajeto até o eixo não garante ausência de colisões.

A finalização registra amostras da pose e velocidade em `python/run/docking/<data>/pose_visual.jsonl`. A tolerância após a parada é de 6 cm longitudinal, 8 cm lateral e 6 graus; se a pose não puder ser confirmada dentro desses limites, permanece em HOLD e armado. A nova margem de frenagem é estimada e ainda exige ensaio em movimento no Gazebo.

## Saída do contêiner de ré

Com Gazebo, câmera e MAVProxy ativos, deixe o rover parado em **HOLD**, dentro da estação e de frente para o QR cadastrado. Se o monitor de entrada estiver aberto na porta UDP 14551, encerre-o com Ctrl+C antes de iniciar a saída; os dois programas não podem ocupar essa porta ao mesmo tempo. Em outro terminal:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/ardupilot/Rover
/home/grin/.pyenv/versions/3.8.10/bin/python3.8 python/docking/sair_container.py
```

A rotina exige telemetria SITL, leitura do QR da estação `container_gravel_plane` e uma pose visual compatível com o interior e o alinhamento do contêiner. Se estiver desarmado, solicita armamento normal; se o ArduPilot recusar, não prossegue. Em GUIDED, recua seguindo o QR e usa a variação da posição visual para medir os 2 m. Em seguida solicita HOLD e confirma parada, distância entre 1,85 e 2,15 m e alinhamento antes de girar. O giro de 180° usa o yaw do ArduPilot; ao terminar, solicita HOLD e desarma. O programa não altera a missão.

Para outro cadastro, use `--estacao NOME`. `--velocidade` admite até 0,12 m/s e `--timeout` ajusta o tempo de cada etapa. Ele salva os valores anteriores de `OA_TYPE` e `AVOID_ENABLE` em `python/run/docking/saida_<data>/parametros_anteriores.json`, solicita desativar o desvio para a passagem estreita e tenta restaurar os valores ao terminar ou interromper. `OA_TYPE` pode exigir reinicialização para afetar o planejador: confirme o comportamento da configuração ativa. A rotina não valida obstáculos atrás do rover; mantenha a área de recuo e giro livre.

Se o QR sumir durante o recuo ou o alinhamento ultrapassar o limite, solicita HOLD sem iniciar o giro. Se o modo for alterado externamente, interrompe a rotina. Ctrl+C também interrompe e tenta parar o rover. A estimativa depende da câmera e da calibração simuladas; os testes realizados cobrem a lógica geométrica, não uma saída completa em movimento no Gazebo. Não use no rover real sem adaptação e validação.
