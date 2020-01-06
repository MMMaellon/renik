#include "register_types.h"

#include "core/class_db.h"
#include "renik.h"

void register_renik_types() {
	ClassDB::register_class<RenIK>();
}

void unregister_renik_types() {
	// Nothing to do here in this example.
}