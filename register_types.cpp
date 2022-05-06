#include "register_types.h"

#include "core/object/class_db.h"
#include "renik.h"
#include "renik/renik_chain.h"
#include "renik/renik_limb.h"

#ifdef DEBUG_MEMORY_ALLOC
#include "renTest\renTest.h"
#endif

void initialize_renik_module(ModuleInitializationLevel p_level) {
  if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
    return;
  }
  ClassDB::register_class<RenIK>();
  ClassDB::register_class<RenIKLimb>();
  ClassDB::register_class<RenIKChain>();
#ifdef RENIK_UNIT_TEST_H
  printf("Unit Testing RenIK Enabled\n");
  ClassDB::register_class<RenIKTest>();
  RenIKTest::test();
#endif
}

void uninitialize_renik_module(ModuleInitializationLevel p_level) {
  if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
    return;
  }
}
