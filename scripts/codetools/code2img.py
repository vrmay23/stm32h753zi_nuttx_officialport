#!/usr/bin/env python3
import numpy as np
from PIL import Image
import re
import os

def convert():
    path = "/home/vinicius/git/nuttxspace/stm32h753zi/apps/hmi_manager/uiux/assets/img_main_cluster.c"
    output_png = "preview_cluster.png"
    
    with open(path, 'r') as f:
        content = f.read()

    # Pega qualquer hex (0xXX ou 0xXXXX)
    hex_strs = re.findall(r'0x[0-9a-fA-F]+', content)
    
    # Converte para int (Python lida com qualquer tamanho aqui)
    vals = [int(x, 16) for x in hex_strs]
    
    # Se o valor 800 apareceu, o array é uint16_t (RGB565 direto)
    # Vamos pegar os últimos 153.600 valores (geralmente o array de pixels fica no fim do arquivo .c do LVGL)
    if len(vals) >= 153600:
        # Pega os últimos 153600 elementos para evitar pegar lixo do cabeçalho
        data_16 = np.array(vals[-153600:], dtype=np.uint16)
        print(f"Processando {len(data_16)} pixels como uint16_t...")
    else:
        print(f"Erro: Apenas {len(vals)} elementos encontrados. Preciso de 153600.")
        return

    # Conversão RGB565 -> RGB888
    r = ((data_16 >> 11) & 0x1F) << 3
    g = ((data_16 >> 5) & 0x3F) << 2
    b = (data_16 & 0x1F) << 3
    
    rgb888 = np.stack([r, g, b], axis=-1).astype(np.uint8)
    
    try:
        img_data = rgb888.reshape((320, 480, 3))
        img = Image.fromarray(img_data)
        img.save(output_png)
        print(f"SUCESSO: {output_png} gerado.")
    except Exception as e:
        print(f"Erro no reshape: {e}")

if __name__ == "__main__":
    convert()
