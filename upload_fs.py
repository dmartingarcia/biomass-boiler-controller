#!/usr/bin/env python3

import os
import sys
import subprocess
import shutil
from pathlib import Path


def main():
    print("Script para preparar datos para LittleFS en ESP32")

    # Obtener la ruta del proyecto
    project_path = Path(os.path.dirname(os.path.abspath(__file__)))
    data_path = project_path / "data"

    # Verificar que existe el directorio data
    if not data_path.exists():
        print(f"Error: El directorio {data_path} no existe.")
        print("Creando directorio data...")
        data_path.mkdir(exist_ok=True)

    print(f"Usando directorio de datos: {data_path}")

    # Buscar PlatformIO en diferentes ubicaciones
    platformio_paths = [
        "platformio",  # Comando global
        "pio",  # Alias común
        "/usr/local/bin/platformio",
        "/usr/local/bin/pio",
        os.path.expanduser("~/.platformio/penv/bin/platformio"),
        os.path.expanduser("~/.platformio/penv/bin/pio"),
        # Para VS Code en macOS
        os.path.expanduser(
            "~/.vscode/extensions/platformio.platformio-ide-*/penv/bin/platformio"
        ),
        os.path.expanduser(
            "~/.vscode/extensions/platformio.platformio-ide-*/penv/bin/pio"
        ),
    ]

    pio_cmd = None
    for path in platformio_paths:
        # Para el último caso con comodín, buscar coincidencias
        if "*" in path:
            import glob

            for match in glob.glob(path):
                if os.path.exists(match) and os.access(match, os.X_OK):
                    pio_cmd = match
                    break
        elif os.path.exists(path) and os.access(path, os.X_OK):
            pio_cmd = path
            break
        else:
            try:
                # Intentar ejecutar el comando
                subprocess.run([path, "--version"], capture_output=True, check=True)
                pio_cmd = path
                break
            except (subprocess.SubprocessError, FileNotFoundError):
                continue

    if not pio_cmd:
        print("Error: No se pudo encontrar PlatformIO.")
        print(
            "Por favor, instale PlatformIO o proporcione manualmente la ruta al ejecutable."
        )

        # Preguntar al usuario si quiere continuar con un comando personalizado
        custom_cmd = input(
            "¿Desea proporcionar un comando personalizado para compilar y subir? (s/n): "
        )
        if custom_cmd.lower() == "s":
            pio_cmd = input("Introduzca el comando completo (ej: /ruta/a/platformio): ")
        else:
            sys.exit(1)

    print(f"Usando comando PlatformIO: {pio_cmd}")

    # Preparar y subir el sistema de archivos
    print("\nPreparando y subiendo sistema de archivos LittleFS...")

    try:
        # Formatear el sistema de archivos (este paso es importante para resolver problemas de montaje)
        print("Construyendo sistema de archivos...")
        buildfs_cmd = [pio_cmd, "run", "--target", "buildfs"]
        subprocess.run(buildfs_cmd, cwd=project_path, check=True)

        print("Subiendo sistema de archivos...")
        uploadfs_cmd = [pio_cmd, "run", "--target", "uploadfs"]
        subprocess.run(uploadfs_cmd, cwd=project_path, check=True)

    except subprocess.SubprocessError as e:
        print(f"Error al ejecutar PlatformIO: {e}")
        print(
            "Comando que falló:",
            " ".join(buildfs_cmd if "buildfs" in str(e) else uploadfs_cmd),
        )
        sys.exit(1)

    print("\nSistema de archivos LittleFS subido correctamente.")
    print("¡Ahora puedes acceder a tu interfaz web!")


if __name__ == "__main__":
    main()
