/* See: http://www.embedded-bits.co.uk/2011/writeanmlo/ */

.arm
.extern inmate_main

.section ".boot", "ax"
.global vectors
.align 5
vectors:
	b	vector_reset
	b	vector_undef
	b	vector_svc
	b	vector_pabt
	b	vector_dabt
	b	vector_unused
	b	vector_irq
	b	vector_fiq

.macro vector, name
	.weak vector_\name
	vector_\name:
	b	.
.endm
	vector undef
	vector svc
	vector pabt
	vector dabt
	vector unused
	vector irq
	vector fiq

vector_reset:
	ldr	r0, =vectors
	mcr	p15, 0, r0, c12, c0, 0	@ VBAR

	ldr	sp, =stack_top /* setup stack for application */

	b	inmate_main

	.ltorg
