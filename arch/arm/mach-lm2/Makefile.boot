# Those numbers are used only by the non-DT V2P-CA9 platform
# The DT-enabled ones require CONFIG_AUTO_ZRELADDR=y
#   zreladdr-y	+= 0x05008000
#params_phys-y	:= 0x05000100
#initrd_phys-y	:= 0x05000000
   zreladdr-y	+= 0x00008000
params_phys-y	:= 0x00000100
initrd_phys-y	:= 0x00000000
