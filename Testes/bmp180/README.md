# Projeto STM32 - BMP180 Drone

##  ITENS
- **Sensor:** BMP180
- **Placa:** STM32F103C8T6 (Blue Pill)
- **Cabos jumper**

### Conexão Física BMP180 ↔ STM32
| BMP180 | STM32 (Blue Pill) |
|--------|-------------------|
| VCC    | 3.3V              |
| GND    | GND               |
| SDA    | PB7 (I2C1_SDA)    |
| SCL    | PB6 (I2C1_SCL)    |

## Arquivos Principais
- `Core/Src/main.c` → Programa principal
- `Core/Inc/BMP180.h` e `Core/Src/BMP180.c` → Biblioteca do sensor BMP180

## Inicializações Importantes
Na função `main()`: 
1. `BMP180_Start();` → Calibração do sensor BMP180.  
2. `Altura_inicial = BMP180_GetAlt(0);` → Leitura da altitude inicial (referência para o voo).

##  Loop Principal
No loop:
- `Temperatura = BMP180_GetTemp();` → Mede a temperatura.  
- `Pressao = BMP180_GetPress(0);` → Mede a pressão atmosférica.  
- `Altitude = BMP180_GetAlt(0);` → Mede a altitude atual.  
- `Altura = Altitude - Altura_inicial;` → Calcula a altura relativa em relação ao ponto inicial.  
- `HAL_Delay(1000);` → Aguarda 1 segundo entre as medições.  
Variáveis em formato (float)

---

## Resumo
Este projeto utiliza o **STM32F103C8T6(blue pill)** e o sensor **BMP180** via barramento **I2C**.  
Medindo **temperatura, pressão e altitude**, ajustando a altitude relativa em relação ao ponto inicial (quando o drone é ligado).

## REFERÊNCIAS
https://www.youtube.com/watch?v=2Fbkpzisjts