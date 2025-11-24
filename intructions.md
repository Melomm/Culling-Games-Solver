# GUIA COMPLETO DE INSTALACAO NO WINDOWS

Este guia instala todas as dependências necessárias para compilar e executar o simulador e os solvers nativos (maze_solvers) no Windows 10/11, seguindo a mesma hierarquia utilizada no workspace:

```
C:\dev\cg_ws\
 ├── build\
 ├── install\
 ├── log\
 └── src\
      ├── culling_games\
      └── maze_solvers\
```

---

# 0. Premissas

* Windows 10/11 64-bit
* ROS2 instalado em: `C:\humble\ros2-windows\`
* Workspace em: `C:\dev\cg_ws\`

---

# 1. Instalar Visual Studio 2022 (C++ Build Tools)

O ROS2 e o colcon no Windows exigem o compilador MSVC.

Baixe Visual Studio 2022 Build Tools:

[https://visualstudio.microsoft.com/downloads/](https://visualstudio.microsoft.com/downloads/)

Selecione a opção:
**Build Tools for Visual Studio 2022**

Na tela de workloads, marque:

* **Desktop development with C++**
* Inclua os componentes:

  * MSVC v143 build tools
  * Windows 10/11 SDK
  * C++ CMake tools for Windows
  * C++ ATL for latest build tools (opcional, mas recomendado)
  * C++ MFC (opcional)

Instale e reinicie o PC se solicitado.

---

# 2. Instalar Chocolatey (Admin PowerShell)

Abra PowerShell como Administrador e execute:

```powershell
Set-ExecutionPolicy Bypass -Scope Process -Force
[System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072
iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))
```

---

# 3. Pré-requisitos via Chocolatey

```powershell
choco install -y git cmake
choco install -y vcredist2013 vcredist140
```

---

# 4. Instalar Python 3.8.3 (64-bit)

Baixe:

[https://www.python.org/downloads/release/python-383/](https://www.python.org/downloads/release/python-383/)

Marque:

* Add to PATH
* Install for all users

Instale em:

```
C:\Python38\
```

Teste:

```bat
python --version
```

---

# 5. Instalar OpenSSL 1.1.1 (Win64)

Baixe:

[https://slproweb.com/products/Win32OpenSSL.html](https://slproweb.com/products/Win32OpenSSL.html)

Instale em:

```
C:\Program Files\OpenSSL-Win64\
```

Adicione ao PATH do sistema:

```
C:\Program Files\OpenSSL-Win64\bin\
```

---

# 6. Instalar ROS2 Humble (binário)

Baixe:

```
ros2-humble-20240523-windows-release-amd64.zip
```

De:

[https://github.com/ros2/ros2/releases/tag/release-humble-20240523](https://github.com/ros2/ros2/releases/tag/release-humble-20240523)

Extraia para:

```
C:\humble\ros2-windows\
```

---

# 7. Instalar colcon (Python 3.8)

```bat
set PATH=C:\Python38;C:\Python38\Scripts;%PATH%
python -m pip install -U pip
python -m pip install -U colcon-common-extensions
```

---

# 8. Instalar dependências Python necessárias para o build de pacotes ROS2

```bat
set PATH=C:\Python38;C:\Python38\Scripts;%PATH%
python -m pip install -U setuptools packaging
python -m pip install numpy pyyaml lark netifaces empy pygame
```

---

# 9. Preparar o workspace

Criar diretórios:

```bat
mkdir C:\dev\cg_ws
mkdir C:\dev\cg_ws\src
```

Copiar para `src\`:

```
cg\
cg_interfaces\
maze_solvers\
```

Estrutura final:

```
C:\dev\cg_ws\
 ├── src\
 │    ├── cg\
 │    ├── cg_interfaces\
 │    └── maze_solvers\
 ├── build\
 ├── install\
 └── log\
```

---

# 10. Compilar o workspace

Abra:

**x64 Native Tools Command Prompt for VS 2022**

Então execute:

```bat
set PATH=C:\Python38;C:\Python38\Scripts;%PATH%
call C:\humble\ros2-windows\local_setup.bat

cd C:\dev\cg_ws
colcon build --cmake-args ^
  "-DPython3_EXECUTABLE=C:/Python38/python.exe" ^
  "-DPYTHON_EXECUTABLE=C:/Python38/python.exe"

call install\local_setup.bat
```

Os executáveis C++ dos solvers são gerados na pasta `install`.

---

# 11. Executar o simulador

Terminal 1 (Native Tools):

```bat
set PATH=C:\Python38;C:\Python38\Scripts;%PATH%
call C:\humble\ros2-windows\local_setup.bat
call C:\dev\cg_ws\install\local_setup.bat
ros2 run cg maze
```

---

# 12. Executar os solvers nativos C++ (maze_solvers)

Terminal 2 (Native Tools):

```bat
set PATH=C:\Python38;C:\Python38\Scripts;%PATH%
call C:\humble\ros2-windows\local_setup.bat
call C:\dev\cg_ws\install\local_setup.bat
```

Parte 1:

```bat
ros2 run maze_solvers solve_part1
```

Parte 2:

```bat
ros2 run maze_solvers solve_part2
```