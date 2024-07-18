# autonomous-fly

configurar o ambiente

https://github.com/mustafa-gokce/ardupilot-software-development/blob/main/environment-setup/quickstart-windows.md

Abra a pasta \ardu-sim\ em um terminal cmd e rode:
$ arducopter -w -S --model + --speedup 1 --defaults parameters/copter.parm -I0

Abra a pasta \ardu-sim\ em outro terminal rode:
$ mavproxy --master tcp:127.0.0.1:5760 --out 127.0.0.1:14550 --out 127.0.0.1:14560

para abrir o mapa 
> module load map




OBS: ip padrão do drone 127.0.0.1:14550