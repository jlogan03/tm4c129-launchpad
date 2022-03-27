/// RX buffer descriptor field definitions
/// 
/// "Descriptor" structure is the software interface with the direct memory access controller.
/// Hardware interprets Descriptors as members of a linked list with a particular format.
/// Each Descriptor contains a pointer to its buffer segment, a pointer to the next descriptor,
/// and information about how the content of the buffer should be interpreted
///
/// Assumes we are using 8-word descriptors ("alternate descriptor size" peripheral config).