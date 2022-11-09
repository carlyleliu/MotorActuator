#!/bin/bash
st-flash erase
st-flash write build/Robotchissis.bin  0x08000000

