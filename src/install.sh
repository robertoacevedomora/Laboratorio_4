#!/bin/sh

echo "Instalando algunas cosas desde repositorio..."
sudo apt update
sudo apt install xxd
sudo apt install python3 python3-pip

echo "Instalando modulos con pip3..."
# Setup environment
pip3 install pandas numpy matplotlib
pip3 install tensorflow

