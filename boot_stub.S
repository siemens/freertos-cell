/* See: http://www.embedded-bits.co.uk/2011/writeanmlo/ */

.arm
.extern inmate_main

.set SYS_MODE,	0x1f
.set SVC_MODE,	0x13
.set IRQ_MODE,	0x12
.set MODE_MASK,	0x1f

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

	bl	_install_irq_stack
	ldr	sp, =stack_top /* setup stack for application */

	b	inmate_main

_install_irq_stack:
      mrs     r0, cpsr          /* Switch to IRQ mode */
      and     r1, r0, #MODE_MASK/* Save old mode in r1 */
      bic     r0, r0, r1        /* Clear mode bits */
      orr     r0, r0, #IRQ_MODE /* Enter IRQ mode */
      msr     cpsr, r0
      ldr     sp, =irq_stack_top   /* Set irq stack */
      mrs     r0, cpsr          /* Switch back to old mode */
      bic     r0, r0, #MODE_MASK
      orr     r0, r0, r1        /* Restore old mode from r1 */
      msr     cpsr, r0
      bx      lr

	.ltorg
