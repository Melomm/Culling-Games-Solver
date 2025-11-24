# Comandos rapidos de compilar o solver e executar

## PASSO 1: COMPILAR

Abra **x64 Native Tools Command Prompt for VS 2022**:

```batch
call C:\humble\ros2-windows\local_setup.bat
cd C:\dev\cg_ws
colcon build --packages-select maze_solvers
call install\local_setup.bat
```

## PASSO 2: EXECUTAR

### Terminal 1 - Simulador

```batch
call C:\humble\ros2-windows\local_setup.bat
call C:\dev\cg_ws\install\local_setup.bat
ros2 run cg maze
```

### Terminal 2

### Part 1 (get map)

```batch
call C:\humble\ros2-windows\local_setup.bat
call C:\dev\cg_ws\install\local_setup.bat
ros2 run maze_solvers solve_part1
```

### Part 2 (mapping)

```batch
ros2 run maze_solvers solve_part2
```