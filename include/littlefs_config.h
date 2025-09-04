#ifndef LITTLEFS_CONFIG_H
#define LITTLEFS_CONFIG_H

// Configuración personalizada para LittleFS
// Estos valores son específicos para ESP32-S2

// Tamaño de bloque: 4KB (valor típico para ESP32)
#define LFS_BLOCK_SIZE      4096

// Tamaño de lectura de caché: 512 bytes
#define LFS_CACHE_SIZE      512

// Tamaño mínimo de directorio: 256 bytes (suficiente para la mayoría de los casos)
#define LFS_LOOKAHEAD_SIZE  256

// Habilitar protección contra pérdida de energía (recomendado)
#define LFS_PROG_SIZE       64

#endif // LITTLEFS_CONFIG_H
