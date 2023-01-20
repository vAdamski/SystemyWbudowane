.thumb

.equ RCC_BASE ,              0x40021000	@(AHBPERIPH_BASE + 0x00001000)
.equ GPIOC_BASE,             0x48000800	@ (AHB2PERIPH_BASE + 0x00000800)
.equ ECC_AHB2ENR_OFFSET, 	 0x4c
.equ BSSR,					 0x18
.equ BSR,       			 0x28

.global main

.thumb_func

delay:
    ldr r0, =1000000         	@ załaduj wartość 1000000 do rejestru r0
    delay_loop:
    sub r0, r0, #1            	@ zmiejsz r0 o 1
    bne delay_loop           	@ skocz do delay1_loop jeśli r0 nie jest równe 0
    bx lr                     	@ zwróć się z funkcji


main:
    @ ustawienie zegara dla peryferii GPIOC w rejestrze RCC
    ldr r0, =(RCC_BASE + ECC_AHB2ENR_OFFSET)  @ załaduj adres rejestru RCC_AHB2ENR do r0
    ldr r1, =(1 << 2)                          @ załaduj wartość (1 << 2) do r1
    str r1, [r0]                               @ zapisz wartość r1 pod adresem z r0

    @ ustawienie pinu GPIOC 6 jako wyjścia
    ldr r0, =(GPIOC_BASE + 0x00)               @ załaduj adres rejestru GPIOC_MODER do r0
    ldr r1, =(1 << (6*2))                      @ załaduj wartość (1 << (6*2)) do r1
    str r1, [r0]                               @ zapisz wartość r1 pod adresem z r0

blinking:
    @ Przypisanie stanu wysokiego do PC9
    ldr r0, =(GPIOC_BASE + BSSR)               @ załaduj adres rejestru GPIOC_BSSR do r0
    ldr r1, =(1 << 6)                          @ załaduj wartość (1 << 6) do r1
    str r1, [r0]                               @ zapisz wartość r1 pod adresem z r0

    @ opóźnienie
    bl delay                                  @ wywołaj funkcję delay

    @ Przypisanie stanu niskiego do PC9
    ldr r0, =(GPIOC_BASE + BSR)                @ załaduj adres rejestru GPIOC_BSR do r0
    ldr r1, =(1 << 6)                          @ załaduj wartość (1 << 6) do r1
    str r1, [r0]                               @ zapisz wartość r1 pod adresem z r0

    @ opóźnienie
    bl delay                                  @ wywołaj funkcję delay

    b blinking                                 @ skocz do etykiety blinking, powodując powtórzenie pętli
