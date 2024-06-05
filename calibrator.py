#!/usr/bin/env python3
"""Pymodbus synchronous client example.

An example of a single threaded synchronous client.

usage: simple_sync_client.py

All options must be adapted in the code
The corresponding server must be started before e.g. as:
    python3 server_sync.py
"""

# --------------------------------------------------------------------------- #
# import the various client implementations
# --------------------------------------------------------------------------- #
import struct
from PySimpleGUI import PySimpleGUI as sg
import pymodbus.client as ModbusClient
from pymodbus.constants import Endian
from pymodbus import (
    ExceptionResponse,
    Framer,
    ModbusException,
    pymodbus_apply_logging_config,
)

register_dict = {
    "Tensão A": [10,2],
    "Tensão B": [12,3],
    "Tensão C": [14,4],
    "Corrente A": [20,5],
    "Corrente B": [22,6],
    "Corrente C": [24,7],
    "Potência Ativa A": [36,8],
    "Potência Ativa B": [38,9],
    "Potência Ativa C": [40,10],
    "Defasagem A": [
        [36,8],  # Potenc Ativa, Scala Ativa
        [44,11], # Potenc Reati, Defasagem
        [52,60]  # Potenc Apare, FP
    ],
    "Defasagem B": [
        [38,9],  # Potenc Ativa, Scala Ativa
        [46,12], # Potenc Reati, Defasagem
        [54,62]  # Potenc Apare, FP
    ],
    "Defasagem C": [
        [40,10],  # Potenc Ativa, Scala Ativa
        [48,13], # Potenc Reati, Defasagem
        [56,64]  # Potenc Apare, FP
    ],
}

layout_connect = [
    [sg.Text('Informe a porta serial (e.g. "COM3")'),sg.Input(key='PORTA_SERIAL')],
    [sg.Text('Informe o baud rate (e.g. 9600)'),sg.Input(key='BAUD_RATE')],
    [sg.Text('Informe o endereço do dispositivo (e.g. 104)'),sg.Input(key='SLAVE_ADDR')],
    [sg.Button('Conectar'),sg.Text("",key='STATUS_CONN')]
]

layout_main = [
    [sg.Text('CALIBRATOR')],
    [sg.Button('Tensão A'),sg.Button('Corrente A'),sg.Button('Defasagem A'),sg.Button('Potência Ativa A')],
    [sg.Button('Tensão B'),sg.Button('Corrente B'),sg.Button('Defasagem B'),sg.Button('Potência Ativa B')],
    [sg.Button('Tensão C'),sg.Button('Corrente C'),sg.Button('Defasagem C'),sg.Button('Potência Ativa C')],
    # [sg.Text('Senha'), sg.Input(key='senha',password_char='*')],
]

layout_scale = [
    [sg.Text("",key='SCALE',justification='c'),sg.Button('Atualizar',key='UPDATE1')],
    [sg.Text('Valor Lido'),sg.Text("",key='VAL_LIDO')],
    [sg.Text('Valor Ideal'),sg.Input(key='VAL_IDEAL'),sg.Button('Enviar',key='SEND_SCALE')],
    [sg.Button('Voltar',key='BACK1')]
]

layout_phase = [
    [sg.Text("",key='PHASE',justification='c'),sg.Button('Atualizar',key='UPDATE2')],
    [sg.Text('Potencia Ativa'),sg.Text("",key='VAL_LIDO_ATV')],
    [sg.Text('Potencia Reativa'),sg.Text("",key='VAL_LIDO_RTV')],
    [sg.Text('Potencia Aparente'),sg.Text("",key='VAL_LIDO_APA')],
    [sg.Text('Fator de Potência'),sg.Text("",key='VAL_LIDO_FP')],
    [sg.Text('Defasagem atual'),sg.Text("",key='VAL_LIDO_DEF')],
    [sg.Text('Nova defasagem'),sg.Input(key='NOVA_FASE'),sg.Button('Enviar',key='SEND_PHASE')],
    [sg.Button('Voltar',key='BACK2')]
]

layout = [
    [
    sg.Column(layout_connect,key='-COL1-'),
    sg.Column(layout_main,key='-COL2-',visible=False),
    sg.Column(layout_scale,key='-COL3-',visible=False),
    sg.Column(layout_phase,key='-COL4-',visible=False)
    ]
]

def unsigned_to_signed_16bit(n):
    # Ensure n is within 16-bit unsigned integer range
    n = n & 0xFFFF
    # Check if the sign bit (15th bit) is set
    if n & 0x8000:
        # If sign bit is set, convert to negative value
        return n - 0x10000
    else:
        # If sign bit is not set, return n as is
        return n

def signed_to_unsigned_16bit(n):
    # Ensure n is within 16-bit signed integer range
    n = n & 0xFFFF
    # If n is negative, convert it to unsigned 16-bit
    if n < 0:
        return n + 0x10000
    else:
        return n

def registers_to_float(reg_list):
    reg_list = (reg_list[0]<<16 | reg_list[1])
    return struct.unpack('f', reg_list.to_bytes(4))[0]

def register_to_int(reg):
    bytes_data = struct.pack('<H', reg[0])
    reversed_bytes = bytes_data[::-1]
    return struct.unpack('<H', reversed_bytes)[0]

def get_hold_param(cli,addr,reg):
    print("get and verify data")
    try:
        rr = cli.read_holding_registers(reg, 1, slave=addr)
    except ModbusException as exc:
        print(f"Received ModbusException({exc}) from library")
        cli.close()
    if rr.isError():
        print(f"Received Modbus library error({rr})")
        cli.close()
        return None
    if isinstance(rr, ExceptionResponse):
        print(f"Received Modbus library exception ({rr})")
        # THIS IS NOT A PYTHON EXCEPTION, but a valid modbus message
        cli.close()
    
    return rr.registers  

def set_hold_param(cli,addr,reg,data):
    print("get and verify data")
    try:
        rr = cli.write_register(reg, data, slave=addr)
    except ModbusException as exc:
        print(f"Received ModbusException({exc}) from library")
        cli.close()
    if rr.isError():
        print(f"Received Modbus library error({rr})")
        cli.close()
        return None
    if isinstance(rr, ExceptionResponse):
        print(f"Received Modbus library exception ({rr})")
        # THIS IS NOT A PYTHON EXCEPTION, but a valid modbus message
        cli.close()
    
    return rr.registers  


def get_input_measure(cli,addr,reg):
    print("get and verify data")
    try:
        rr = cli.read_input_registers(reg, 2, slave=addr)
    except ModbusException as exc:
        print(f"Received ModbusException({exc}) from library")
        cli.close()
    if rr.isError():
        print(f"Received Modbus library error({rr})")
        cli.close()
        return None
    if isinstance(rr, ExceptionResponse):
        print(f"Received Modbus library exception ({rr})")
        # THIS IS NOT A PYTHON EXCEPTION, but a valid modbus message
        cli.close()
    
    return rr.registers

def main():
    """Run sync client."""
    # activate debugging
    pymodbus_apply_logging_config("DEBUG")
    print("connect to server")

    #Janela
    janela = sg.Window('Calibrator', layout, element_justification='c')
    global nome_param
    global val_lido

    #Ler os eventos
    while True:
        eventos, valores = janela.read()
        if eventos in (None, 'Exit'):
            break
        if eventos == 'Conectar':
            janela['STATUS_CONN'].update(value='')

            global client
            client = ModbusClient.ModbusSerialClient(
                valores['PORTA_SERIAL'],
                framer=Framer.RTU,
                # timeout=10,
                # retries=3,
                # retry_on_empty=False,
                # strict=True,
                baudrate=int(valores['BAUD_RATE']),
                bytesize=8,
                parity="N",
                stopbits=1,
                # handle_local_echo=False,
                # Set the desired byte order (little endian)
                # endian = Endian.LITTLE 
            )
            client.connect()
            serial_num = get_input_measure(client,int(valores['SLAVE_ADDR']),0)
            if(serial_num == None):
                janela['STATUS_CONN'].update(value='Falha na conexão!')
                print('Error')
            else:
                janela['-COL1-'].update(visible=False)
                janela['-COL2-'].update(visible=True)

        if 'Tensão' in eventos or 'Corrente' in eventos or 'Potência' in eventos:
            nome_param = eventos

            janela['-COL2-'].update(visible=False)
            janela['-COL3-'].update(visible=True)

            janela['SCALE'].update(value=eventos)
            measure = get_input_measure(client,int(valores['SLAVE_ADDR']),register_dict[nome_param][0])
            if measure != None:
                val_lido = registers_to_float(measure)
                janela['VAL_LIDO'].update(value=val_lido)
        
        if 'Defasagem' in eventos:
            nome_param = eventos

            janela['-COL2-'].update(visible=False)
            janela['-COL3-'].update(visible=False)            
            janela['-COL4-'].update(visible=True)    

            janela['PHASE'].update(value=eventos)

            reg_atv = get_input_measure(client,int(valores['SLAVE_ADDR']),register_dict[nome_param][0][0])
            reg_rtv = get_input_measure(client,int(valores['SLAVE_ADDR']),register_dict[nome_param][1][0])
            reg_apa = get_input_measure(client,int(valores['SLAVE_ADDR']),register_dict[nome_param][2][0])
            reg_fpo = get_input_measure(client,int(valores['SLAVE_ADDR']),register_dict[nome_param][2][1])
            param = get_hold_param(client,int(valores['SLAVE_ADDR']),register_dict[nome_param][1][1])

            if reg_atv != None and reg_rtv != None and reg_apa != None and reg_fpo != None and param != None:
                pot_atv = registers_to_float(reg_atv)
                pot_rtv = registers_to_float(reg_rtv)
                pot_apa = registers_to_float(reg_apa)
                fp = registers_to_float(reg_fpo)

                defasagem = unsigned_to_signed_16bit(register_to_int(param))
                defasagem_us = (defasagem << 3) / 8.338608
                janela['VAL_LIDO_DEF'].update(value=defasagem_us)

                janela['VAL_LIDO_ATV'].update(value=pot_atv)
                janela['VAL_LIDO_RTV'].update(value=pot_rtv)
                janela['VAL_LIDO_APA'].update(value=pot_apa)
                janela['VAL_LIDO_FP'].update(value=fp)

        if 'BACK' in eventos:
            janela['-COL2-'].update(visible=True)
            janela['-COL3-'].update(visible=False)
            janela['-COL4-'].update(visible=False)

        if 'UPDATE' in eventos:
            if 'Tensão' in nome_param or 'Corrente' in nome_param or 'Potência' in nome_param:
                measure = get_input_measure(client,int(valores['SLAVE_ADDR']),register_dict[nome_param][0])
                val_lido = registers_to_float(measure)
                janela['VAL_LIDO'].update(value=val_lido)

            if 'Defasagem' in nome_param:
                reg_atv = get_input_measure(client,int(valores['SLAVE_ADDR']),register_dict[nome_param][0][0])
                reg_rtv = get_input_measure(client,int(valores['SLAVE_ADDR']),register_dict[nome_param][1][0])
                reg_apa = get_input_measure(client,int(valores['SLAVE_ADDR']),register_dict[nome_param][2][0])
                reg_fpo = get_input_measure(client,int(valores['SLAVE_ADDR']),register_dict[nome_param][2][1])
                param = get_hold_param(client,int(valores['SLAVE_ADDR']),register_dict[nome_param][1][1])

                if reg_atv != None and reg_rtv != None and reg_apa != None and reg_fpo != None and param != None:
                    pot_atv = registers_to_float(reg_atv)
                    pot_rtv = registers_to_float(reg_rtv)
                    pot_apa = registers_to_float(reg_apa)
                    fp = registers_to_float(reg_fpo)

                    defasagem = unsigned_to_signed_16bit(register_to_int(param))
                    defasagem_us = (defasagem << 3) / 8.338608
                    janela['VAL_LIDO_DEF'].update(value=defasagem_us)

                    janela['VAL_LIDO_ATV'].update(value=pot_atv)
                    janela['VAL_LIDO_RTV'].update(value=pot_rtv)
                    janela['VAL_LIDO_APA'].update(value=pot_apa)
                    janela['VAL_LIDO_FP'].update(value=fp)
                
        if eventos == 'SEND_SCALE':
            param = get_hold_param(client,int(valores['SLAVE_ADDR']),register_dict[nome_param][1])
            if(param != None):
                scale = register_to_int(param)
                des_val = valores['VAL_IDEAL']
                if val_lido != 0:
                    Fc = scale * (float(des_val)/val_lido)
                    # new_scale = int(Fc).to_bytes(2, 'little')
                    # new_scale = register_to_int(new_scale)
                    print(f'antigo {scale} novo {Fc}')
                    set_hold_param(client,int(valores['SLAVE_ADDR']),register_dict[nome_param][1],int(Fc))

        if eventos == 'SEND_PHASE':
            nova_fase = float(valores['NOVA_FASE'])
            nova_fase_bytes = signed_to_unsigned_16bit(int(nova_fase * 8.338606) >> 3)
            set_hold_param(client,int(valores['SLAVE_ADDR']),register_dict[nome_param][1][1],int(nova_fase_bytes))
    
    print("close connection")
    client.close()
    janela.close()

main()

a = input('entrada')