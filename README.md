# Laboratorio_4
Repositorio laboratorio 4
Para compilar este ejemplo es necesario tener previamente descargada y compilada
la biblioteca de opencm3. Se debe editar el Makefile para indicar el directorio
donde se encuentra la biblioteca. El ejemplo compilado queda en el directorio build.

Para quemar/subir el binario a la placa se puede utilizar el programa st-flash, por ejemplo:

st-flash --reset write build/firmware.bin 0x8000000

En caso de tener solo un archivo con extension .elf se puede generar el archivo .bin con:

arm-none-eabi-objcopy -O binary firmware.elf  firmware.bin
