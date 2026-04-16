#ifndef TEST_ROUTINE_H
#define TEST_ROUTINE_H

/**
 * @brief Inicia la rutina de prueba para registrar las gráficas de respuesta.
 * La rutina moverá la referencia a 0.2m, esperará 5s, 
 * moverá la referencia a -0.2m, esperará 5s,
 * y finalmente volverá a 0m y detendrá la tarea.
 */
void test_routine_start(void);

#endif // TEST_ROUTINE_H
