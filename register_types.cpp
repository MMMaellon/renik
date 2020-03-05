#include "register_types.h"

#include "core/class_db.h"
#include "renik.h"
#include "renik/renik_limb.h"
#include "renik/renik_chain.h"

#ifdef DEBUG_MEMORY_ALLOC
#include "renTest\renTest.h"
#endif

void register_renik_types() {
	ClassDB::register_class<RenIK>();
	ClassDB::register_class<RenIKLimb>();
	ClassDB::register_class<RenIKChain>();
#ifdef RENIK_UNIT_TEST_H
	printf("Unit Testing RenIK Enabled\n");
	ClassDB::register_class<RenIKTest>();
	RenIKTest::test();
#endif
}

void unregister_renik_types() {
	// Nothing to do here in this example.
}