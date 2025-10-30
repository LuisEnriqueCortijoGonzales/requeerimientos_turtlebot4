# TurtleBot 4 — Setup de Máquina Virtual + ROS 2 Jazzy  
**Versión de Ubuntu:** [Ubuntu 24.04](https://releases.ubuntu.com/24.04)  
**Máquina virtual:** [VirtualBox](https://www.virtualbox.org)  
**Paquetes de ROS:** [ROS 2 Jazzy Debian Packages](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)  
**Manual del TurtleBot4:** [TurtleBot 4 User Manual – Basic Setup](https://turtlebot.github.io/turtlebot4-user-manual/setup/basic.html)  

---

## Índice  
1. Crear máquina virtual con Ubuntu 24.04  
2. Preparar Ubuntu para ROS 2 Jazzy  
3. Instalar ROS 2 Jazzy (incluye `roscore`)  
4. Instalar paquetes específicos para TurtleBot 4  
5. Verificación básica del sistema  
6. Notas y consejos  
7. Referencias  

---

## 1. Crear máquina virtual con Ubuntu 24.04  
1. Instala VirtualBox en tu sistema anfitrión.  
2. Crea una nueva máquina virtual:  
   - Tipo: Linux → Ubuntu (64‑bit)  
   - Memoria RAM: al menos 4 GB (ideal 8 GB)  
   - Disco duro: 20+ GB (depende del uso)  
   - Adjunta la ISO de Ubuntu 24.04 como medio de instalación.  
3. Instala Ubuntu 24.04 dentro de la VM: sigue el asistente de instalación de Ubuntu.  
4. Una vez instalado, asegúrate de instalar las “Guest Additions” de VirtualBox para mejor integración (ratón, pantalla, carpetas compartidas) si lo necesitas.  
5. Actualiza el sistema dentro de la VM:  
   ```bash
   sudo apt update
   sudo apt upgrade -y
   sudo reboot
   ```

---

## 2. Preparar Ubuntu para ROS 2 Jazzy  
1. Asegúrate de que la localización soporte UTF‑8:  
   ```bash
   locale                # revisar que haya UTF‑8
   sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8
   ```
2. Habilita los repositorios requeridos:  
   ```bash
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   ```
3. Añade la clave del repositorio ROS 2 y el repositorio apt:  
   ```bash
   sudo apt update && sudo apt install curl gnupg2 lsb-release
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key      | sudo tee /usr/share/keyrings/ros‑archive-keyring.gpg >/dev/null
   echo "deb [arch=$(dpkg --print‑architecture) signed‑by=/usr/share/keyrings/ros‑archive-keyring.gpg]      http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main"      | sudo tee /etc/apt/sources.list.d/ros2.list
   sudo apt update
   ```

---

## 3. Instalar ROS 2 Jazzy  
1. Instala ROS 2 Jazzy base o escritorio según necesidad:  
   ```bash
   sudo apt install ros‑jazzy‑desktop
   ```  
2. Configura el ambiente automáticamente cada vez que abras un terminal:  
   ```bash
   echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```
3. Verifica la instalación con un nodo de ejemplo:  
   ```bash
   ros2 run demo_nodes_cpp talker
   ```

---

## 4. Instalar paquetes específicos para TurtleBot 4  
1. Instala el paquete TurtleBot4:  
   ```bash
   sudo apt update
   sudo apt install ros‑jazzy‑turtlebot4‑desktop
   ```

---

## 5. Verificación básica del sistema  
1. Asegúrate de que puedes lanzar `roscore` (en ROS 2 se maneja distinto, pero el sistema está listo):  
   ```bash
   source /opt/ros/jazzy/setup.bash
   ros2 topic list
   ```
2. Verifica que el paquete TurtleBot4 esté correctamente instalado:  
   ```bash
   ros2 pkg list | grep turtlebot4
   ```

---

## 6. Notas y consejos  
- Usa **Ubuntu 24.04** ya que ROS 2 Jazzy lo soporta oficialmente.  
- Si tu máquina no es muy potente, asigna suficiente memoria/vCPU.  
- Revisa dependencias si aparecen errores durante la instalación.  

---
