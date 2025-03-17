INC := app/include FreeRTOS-Kernel/include
INC += FreeRTOS-Kernel/portable/GCC/ARM_CM3

tags :
	etags app/*.cpp app/include/*.h FreeRTOS-Kernel/*.c \
           FreeRTOS-Kernel/include/*.h \
           FreeRTOS-Kernel/portable/GCC/ARM_CM3/*.[ch]

mostlyclean :
clean : mostlyclean
	rm -f TAGS
