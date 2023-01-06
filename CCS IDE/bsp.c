/*
 * bsp.c
 *
 *  Created on: Jan 6, 2023
 *      Author: K.Aladawy
 */


/*
 * "naked" functions do not include any prologue or epilogue -- they are naked.
 * In particular, they do not include operations on the stack for local variables,
 * to save or restore registers, or to return to a calling function.
 */
__attribute__((naked)) void assert_failed (char const *file, int line) {
    /* TBD: damage control */
    //TODO: add your code
}
