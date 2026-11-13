# Ensaio de entrada visual no contêiner

O ensaio foi executado no Gazebo Classic com o modelo ARGO, no mundo `gravel_plane`, e ArduRover SITL isolado na instância 20, usando MAVProxy. O rover foi criado para o ensaio em aproximadamente x=3,50 m, y=-2,00 m, alinhado em direção à abertura. Este ensaio cobre a entrada visual; não testa a missão de aproximação desde um ponto arbitrário.

A primeira tentativa parou em HOLD após uma falha breve de decodificação do QR. A rotina foi ajustada para enviar velocidade zero imediatamente e aguardar até 2 s por uma nova leitura; em uma perda mais longa continua interrompendo e retorna a HOLD.

Na segunda tentativa o QR foi acompanhado em GUIDED e o alvo relativo informado pelo programa passou de aproximadamente 1,31 m à frente para 0,04 m. Ao final, o ArduPilot confirmou HOLD e a restauração dos parâmetros `OA_TYPE=1` e `AVOID_ENABLE=1`. A leitura de estado do Gazebo após a parada indicou x=4,919 m e y=-1,992 m, com velocidade horizontal aproximada de 0,002 m/s e yaw de 1,82° no mundo. O interior do contêiner vai de x=4,5 a 5,5 m e de y=-2,5 a -1,5 m. Considerando a geometria das rodas do modelo, o rover terminou dentro desses limites.

Neste ensaio isolado os checks de armamento foram desativados e o MID360 não foi iniciado; isso foi feito somente no SITL de teste. O programa desabilitou o desvio durante a entrada. Não foi registrado um sensor de contato, portanto a ausência de colisão durante toda a trajetória não foi comprovada. Os dados visuais são específicos da câmera, do QR e da geometria simulados; não comprovam operação em um veículo real.

Log do programa: `Rover/python/run/docking_live_test/entrada_segunda.log`. Configuração de teste: `Rover/python/run/docking_live_test/test.launch` e arquivos de parâmetros no mesmo diretório. Esses arquivos de execução são ignorados pelo Git.

## Ajuste de centralização e orientação

A rotina passou a usar como alvo o centro da estação, com compensação visual calibrada de 0,5335 m em relação ao QR, correção também pela orientação do plano do QR, tolerâncias menores e ajuste contínuo da inclinação da câmera. Um ensaio terminou em x=4,98596 m, y=-1,99604 m e yaw=1,363°. Uma repetição terminou em x=4,99092 m, y=-2,00171 m e yaw=1,109°, contra centro (5, -2) m e orientação desejada 0°. Isso corresponde, na repetição, a erros de 9,1 mm longitudinal, 1,7 mm lateral e 1,1° angular. O rover terminou em HOLD e foi desarmado.

Outra condição inicial foi rejeitada pelo teste de alinhamento visual antes de iniciar o movimento. Esses dois ensaios partiram de posições próximas e não demonstram erro zero nem precisão garantida em condições diferentes. Como nos ensaios anteriores, o SITL era isolado, sem MID360 e com armamento simplificado; não houve medição de contato com as paredes. Log da repetição: `Rover/python/run/docking_live_test/entrada_centro_repeticao.log`.
