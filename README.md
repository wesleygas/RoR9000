# RoR9000
Repositório da matéria de robótica computacional dedicada para o robô R9000



Recursos utilizados:
  cv_camera (instruções pra launch ta no readme do cv_camera, depois eu vejo isso)
  darknet_YOLO (possivelmente com CUDA porém essa coisa ta dando um trabalho desgraçado)
    Por enquanto, é  necessário alterar a __main__ do yolo e hardcode as dimensões do arquivo de camera. Contido no 
    tópico do cv_camera (use rostopic list e rostopic echo [nome do topico] para descobrir, te vira)
*O maldido do cuda bota o instalador dos samples em /usr/local/cuda-9.1/bin/cuda-install-samples-9.1.sh*
