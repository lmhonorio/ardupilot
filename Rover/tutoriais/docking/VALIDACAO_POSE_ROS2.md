> Relatório histórico. A revisão posterior está em [Validação do alinhamento e da velocidade](VALIDACAO_ALINHAMENTO_VELOCIDADE_ROS2.md).

# Validação da entrada visual — 09/11/2026

Corrigida a interrupção causada por saltos na orientação estimada do QR.
O teste final completou a entrada no Gazebo Harmonic, confirmou HOLD e desarmou
o rover pelo MAVProxy. Não houve alteração da missão.

## Alterações

- `pose_tracker.py` estima a orientação da parede em um referencial de rumo fixo,
  compensando o giro do rover com ATTITUDE. Exige pelo menos cinco imagens
  distintas cobrindo 0,6 s, usa consenso temporal e rejeita saltos de orientação
  ou posição incompatíveis com o movimento. Reconstrói o alvo usando a parede
  vertical deste modelo e a distância cadastrada entre o QR e o centro.
- Uma medida rejeitada produz velocidade e giro zero. Não aciona busca pelo GPS
  nem avanço sem leitura. Se não houver recuperação em 10 s, solicita HOLD.
- A passagem para entrada exige alinhamento estável durante 0,8 s. Violações dos
  limites de 20 cm lateral ou 25° param imediatamente; persistência por 1 s
  confirma a falha. Esses limites não foram ampliados.
- O pivô parado usa no mínimo 0,12 rad/s, limitado a 0,18 rad/s. No primeiro teste,
  solicitações menores produziam PWM próximo de 1531/1469 e quase nenhum giro.
  Há histerese na entrada: inicia correção acima de 5° e termina abaixo de 2,5°.
  Esse mínimo não é aplicado à direção durante o avanço.
- A confirmação final exige parada e três imagens válidas cobrindo pelo menos
  0,4 s antes de desarmar. Tolerâncias visuais: 6 cm longitudinal, 8 cm lateral,
  6° de orientação.
- O cadastro usa agora **0,499 m** entre QR e centro, conforme a posição do
  visual `qr_code` no modelo Harmonic. Os 0,5335 m anteriores deslocavam o alvo
  nominal 3,45 cm antes do centro. A execução `--uma-vez` também lê esse cadastro.
- `controle.jsonl` registra medidas brutas, bloqueios, fases, velocidades e erros
  usados nas decisões; `pose_visual.jsonl` mantém as poses filtradas.

## Ensaios

1. **165408:** partiu da posição deixada pelas tentativas anteriores, aproximadamente
   1,7 m antes do alvo. A filtragem eliminou a interrupção de alinhamento e permitiu
   avançar até a entrada. O teste foi interrompido com SIGTERM/HOLD ao identificar
   giro insuficiente perto de 6°. Parâmetros de desvio restaurados.
2. **165534:** após corrigir o pivô, retomou a cerca de 0,5 m do alvo. Completou a
   entrada e desarmou. Pose visual final: 4,6 cm longitudinal, −6,1 cm lateral e
   −3,0°. A comparação com Gazebo revelou a diferença do cadastro do QR, corrigida
   antes do teste seguinte.
3. **165733:** com o rover desarmado em HOLD, reposicionou-se somente o modelo
   simulado para `(3,2; −2,0; 0,1513)`, yaw 0°. Após confirmar SITL e telemetria,
   o rover foi armado normalmente, sem contornar PreArm. Executou a entrada
   completa a partir de 1,8 m do centro, sem carregar missão.

Resultado do último teste, medido diretamente em `/world/gravel_plane/pose/info`:

| Medida | Resultado |
| --- | --- |
| Centro do contêiner | x = 5,000 m; y = −2,000 m |
| Posição final do rover | x = 4,95231 m; y = −2,00001 m |
| Erro longitudinal ao centro | −4,77 cm |
| Erro lateral ao centro | aproximadamente 0 cm |
| Erro de orientação | aproximadamente 0° |
| Estado final MAVLink | HOLD, desarmado |
| Maior erro lateral filtrado durante avanço | 8,83 cm |
| Amostras com salto angular rejeitadas | 4, com parada para nova leitura |

O controlador informou pose final de 4,6 cm longitudinal, 0,2 cm lateral e 0,2°.
A pequena diferença em relação ao Gazebo ilustra que estimativa visual e posição
real do simulador são medidas diferentes. A parada antecede o centro por causa
da margem de frenagem; não significa posicionamento matematicamente exato.

Evidências do teste final:

- [Resumo automático](../../python/run/docking/20261109_165733_781198/validacao.json)
- [Posição publicada pelo Gazebo](../../python/run/docking/20261109_165733_781198/gazebo_pose_final.txt)
- [Comandos e decisões](../../python/run/docking/20261109_165733_781198/controle.jsonl)
- [Telemetria MAVLink](../../python/run/docking/20261109_165733_781198/telemetry.jsonl)
- [Resultado dos testes de software](../../python/run/docking/20261109_165733_781198/unit_tests.txt)

## Testes reproduzíveis

```bash
cd ~/ardupilot/Rover
source ros2/setup.bash
python -m unittest discover -s python/tests -p 'test_*.py'
ROS_DOMAIN_ID=93 python python/tests/smoke_docking_ros2.py
```

Passaram **89 testes de software**, incluindo regressão com poses das falhas,
rejeição de imagens repetidas, compensação do giro, proteção lateral, histerese
e consistência entre cadastro e geometria do modelo. O teste DDS isolado passou
para imagem, calibração, juntas, nuvem e odometria. Nenhum desses testes de
software arma ou movimenta o rover.

## Aplicar e limites da validação

Reinicie o processo Python para carregar a alteração:

```bash
cd ~/ardupilot/Rover
./ros2/service.sh docking-qr
```

O rover foi deixado **dentro do contêiner, em HOLD e desarmado**. Para repetir uma
entrada, primeiro retire/reposicione o rover na simulação. O monitor continua
exigindo GUIDED, armado e estação a menos de 8 m.

Validado no cenário gravel_plane, câmera Samsung 640×360, parede vertical e rover
próximo do eixo da entrada. Aproximações com grandes deslocamentos laterais ou
que levem o QR para fora da câmera, veículo real, câmera diferente e a sequência
completa de missão AUTO ainda exigem ensaios próprios. A alteração não implementa
rastreamento do QR pelo pan nem faz fusão FAST-LIO/câmera.
