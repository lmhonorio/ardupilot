# Tutoriais do Rover

Documentação organizada por componente. Atualização: **13/11/2026**.

- [Cheat sheet em PDF — 12 páginas](CHEAT_SHEET.pdf): comandos de início, operação, diagnóstico e parada de todos os componentes atuais.
- [Cheat sheet em Markdown](CHEAT_SHEET.md): versão editável e fácil de copiar.
- [Mapa dos caminhos antigos e novos](MAPA_DE_MUDANCAS.json): localização dos 44 documentos reunidos aqui.

## Começar por aqui

| Objetivo | Tutorial |
| --- | --- |
| Tudo local: Gazebo, SITL, serviços e RViz | [Início rápido local](simulacao/TUTORIAL_RAPIDO_LOCAL.md) |
| Gazebo/SITL no PC, gerenciamento na Raspberry | [Início rápido remoto](companion/TUTORIAL_RAPIDO_REMOTO.md) |
| MID360 físico + FAST-LIO na Pi, nuvem no PC | [Teste independente na Raspberry](mid360/fast_lio_raspberry/README.md) |
| MID360 físico + FAST-LIO processado no PC | [Teste independente local](mid360/fast_lio_local/README.md) |
| Enviar missão, entrar, recarregar e sair | [Mission Manager](mission_manager/TUTORIAL.md) |
| Acionar entrada/saída sem o gerenciador | [Docking no ROS 2](docking/TUTORIAL_ENTRADA_SAIDA_ROS2.md) |
| Instalar e preparar o rover real | [Comissionamento na Pi 5](companion/INSTALACAO_PI5_ROVER_REAL.md) |

## Como usar esta organização

Os códigos, configurações, logs e arquivos de missão permanecem nos caminhos
originais. A pasta `Rover/simulacoes/` ainda contém exemplos `.waypoints` e
outros dados; os comandos que usam esses arquivos continuam iguais.
Execute os comandos a partir de `~/ardupilot/Rover`, salvo indicação diferente.
Mover os manuais não exige reiniciar os serviços.

Os backups em `Rover/backups/` e a documentação original de bibliotecas externas
não foram movidos. Alguns manuais antigos e relatórios descrevem ROS 1/Gazebo
Classic: use o cheat sheet e os tutoriais ROS 2 para as chamadas atuais. Os
resultados históricos não são uma nova validação da instalação atual.

## Componentes e documentos

### Câmera e observação do QR

- [Leitura da câmera e observação do QR no ROS 2](camera/README.md)

### Leitura do LiDAR

- [Leitura do LiDAR simulado e captura de nuvem](lidar/README.md)

### Simulação e serviços locais

- [DIAGNOSTICO_ROS_ARGO.md](simulacao/DIAGNOSTICO_ROS_ARGO.md)
- [MANUAL_SIMULACAO.md](simulacao/MANUAL_SIMULACAO.md)
- [PASSO_A_PASSO_ROS_FAST_LIO.md](simulacao/PASSO_A_PASSO_ROS_FAST_LIO.md)
- [TUTORIAIS_SERVICOS_SEPARADOS.md](simulacao/TUTORIAIS_SERVICOS_SEPARADOS.md)
- [TUTORIAL_RAPIDO_LOCAL.md](simulacao/TUTORIAL_RAPIDO_LOCAL.md)

### ROS 2: instalação e migração

- [ROS2_JAZZY.md](ros2/ROS2_JAZZY.md)
- [VALIDACAO_ROS2.md](ros2/VALIDACAO_ROS2.md)

### Raspberry, Docker e controle remoto

- [INSTALACAO_PI5_ROVER_REAL.md](companion/INSTALACAO_PI5_ROVER_REAL.md)
- [INSTALACAO_PI5_SIMULACAO.md](companion/INSTALACAO_PI5_SIMULACAO.md)
- [RASPBERRY_DOCKER.md](companion/RASPBERRY_DOCKER.md)
- [TUTORIAL_RAPIDO_REMOTO.md](companion/TUTORIAL_RAPIDO_REMOTO.md)
- [VALIDACAO.md](companion/VALIDACAO.md)
- [VALIDACAO_PI5.md](companion/VALIDACAO_PI5.md)

### Gerenciador de missão e recarga

- [ARQUITETURA.md](mission_manager/ARQUITETURA.md)
- [TESTES.md](mission_manager/TESTES.md)
- [TUTORIAL.md](mission_manager/TUTORIAL.md)

### Docking por QR e LiDAR; cadastro de estações

- [CADASTRO_ESTACOES.md](docking/CADASTRO_ESTACOES.md)
- [DOCKING_LIDAR.md](docking/DOCKING_LIDAR.md)
- [README DOCKING.md](docking/README%20DOCKING.md)
- [RESULTADO_ENTRADA.md](docking/RESULTADO_ENTRADA.md)
- [TUTORIAL_ENTRADA_SAIDA_ROS2.md](docking/TUTORIAL_ENTRADA_SAIDA_ROS2.md)
- [VALIDACAO_ALINHAMENTO_VELOCIDADE_ROS2.md](docking/VALIDACAO_ALINHAMENTO_VELOCIDADE_ROS2.md)
- [VALIDACAO_POSE_ROS2.md](docking/VALIDACAO_POSE_ROS2.md)

### SLAM 3D, FAST-LIO e GLIM

- [SLAM_DOCKING_3D.md](slam/SLAM_DOCKING_3D.md)
- [fast_lio/README.pt-BR.md](slam/fast_lio/README.pt-BR.md)
- [glim/GLIM_MID360.md](slam/glim/GLIM_MID360.md)

### MID360 físico e testes independentes FAST-LIO

- [README.md](mid360/README.md)
- [TESTES.md](mid360/TESTES.md)
- [fast_lio_local/README.md](mid360/fast_lio_local/README.md)
- [fast_lio_local/TESTES.md](mid360/fast_lio_local/TESTES.md)
- [fast_lio_raspberry/README.md](mid360/fast_lio_raspberry/README.md)
- [fast_lio_raspberry/TESTES.md](mid360/fast_lio_raspberry/TESTES.md)

### Missões de teste e sintonia

- [README.pt-BR.md](mission_tune/README.pt-BR.md)
- [VALIDATION.md](mission_tune/VALIDATION.md)

### Lua, alocação e navegação

- [AJUSTE_NAVEGACAO_20261105.md](lua/AJUSTE_NAVEGACAO_20261105.md)
- [README.pt-BR.md](lua/README.pt-BR.md)
- [RESULTADO_SEGUIMENTO_LINHA.md](lua/RESULTADO_SEGUIMENTO_LINHA.md)

### Percepção de obstáculos

- [AUDITORIA.md](obstacle_avoidance/AUDITORIA.md)
- [README.md](obstacle_avoidance/README.md)
- [RESULTADOS.md](obstacle_avoidance/RESULTADOS.md)

### Cenário Gazebo simples

- [ALLOCATION_VALIDATION.md](gazebo_simple/ALLOCATION_VALIDATION.md)
- [README.pt-BR.md](gazebo_simple/README.pt-BR.md)

### Organização dos programas Python

- [README.md](python/README.md)
- [VALIDATION.md](python/VALIDATION.md)

## Atualizar o PDF

Edite `CHEAT_SHEET.md` e execute o gerador. Ele usa ReportLab e as fontes
DejaVu Sans/Mono instaladas no sistema. Para preparar um ambiente isolado:

```bash
cd ~/ardupilot/Rover
python3 -m venv /tmp/rover_docs_pdf_venv
/tmp/rover_docs_pdf_venv/bin/pip install reportlab
/tmp/rover_docs_pdf_venv/bin/python tutoriais/gerar_cheat_sheet.py
```

O PDF contém texto selecionável; os comandos podem ser copiados. As quebras
explícitas de página da versão Markdown mantêm a divisão em 12 páginas.
Depois de editar, confira a paginação e os números do índice.
