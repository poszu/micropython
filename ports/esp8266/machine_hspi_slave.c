#include <stdio.h>
#include "py/nlr.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "py/binary.h"
#include "modmachine.h"
#include "hspi.h"

STATIC mp_obj_t machine_spislave_hello(void) {
    printf("Hello world!\n");
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(machine_spislave_hello_obj, machine_spislave_hello);

// this is the actual C-structure for our new object
typedef struct _machine_spislave_obj_t {
    // base represents some basic information, like type
    mp_obj_base_t base;
    // a member created by us
    uint8_t spi_no;
    bool enabled;
} machine_spislave_obj_t;

STATIC void machine_spislave_print( const mp_print_t *print,
                                  mp_obj_t self_in,
                                  mp_print_kind_t kind ) {
    // get a ptr to the C-struct of the object
    machine_spislave_obj_t *self = MP_OBJ_TO_PTR(self_in);
    // print the number
    printf ("SpiSlave(%u) enabled=%d", self->spi_no, self->enabled ? 1 : 0);
}

STATIC mp_obj_t machine_spislave_enable(mp_obj_t self_in) {
    machine_spislave_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->enabled = true;
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(machine_spislave_enable_obj,
                          machine_spislave_enable);

STATIC mp_obj_t machine_spislave_disable(mp_obj_t self_in) {
    machine_spislave_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->enabled = false;
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(machine_spislave_disable_obj,
                          machine_spislave_disable);


// creating the table of global members
STATIC const mp_rom_map_elem_t machine_spislave_locals_dict_table[] = {
  { MP_ROM_QSTR(MP_QSTR_enable), MP_ROM_PTR(&machine_spislave_enable_obj) },
  { MP_ROM_QSTR(MP_QSTR_disable), MP_ROM_PTR(&machine_spislave_disable_obj) },
};
STATIC MP_DEFINE_CONST_DICT(machine_spislave_locals_dict,
                            machine_spislave_locals_dict_table);

mp_obj_t machine_spislave_make_new( const mp_obj_type_t *type,
                                  size_t n_args,
                                  size_t n_kw,
                                  const mp_obj_t *args ) {
    // this checks the number of arguments (min 1, max 1);
    // on error -> raise python exception
    mp_arg_check_num(n_args, n_kw, 1, 1, true);
    // create a new object of our C-struct type
    machine_spislave_obj_t *self = m_new_obj(machine_spislave_obj_t);
    // give it a type
    self->base.type = &machine_spislave_type;
    // set the member number with the first argument of the constructor
    self->spi_no = mp_obj_get_int(args[0]);
    self->enabled = false;
    spi_slave_init(self->spi_no, 8);
    return MP_OBJ_FROM_PTR(self);
}

// create the class-object itself
const mp_obj_type_t machine_spislave_type = {
    // "inherit" the type "type"
    { &mp_type_type },
     // give it a name
    .name = MP_QSTR_SpiSlave,
     // give it a print-function
    .print = machine_spislave_print,
     // give it a constructor
    .make_new = machine_spislave_make_new,
     // and the global members
    .locals_dict = (mp_obj_dict_t*)&machine_spislave_locals_dict,
};
