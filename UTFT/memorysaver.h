// UTFT Memory Saver
// -----------------
//
// Since most people have only one or possibly two different display modules a lot
// of memory has been wasted to keep support for many unneeded controller chips.
// You now have the option to remove this unneeded code from the library with
// this file.
// By disabling the controllers you don't need you can reduce the memory footprint
// of the library by several Kb.
//
// Uncomment the lines for the displaycontrollers that you don't use to save
// some flash memory by not including the init code for that particular
// controller.

//#define DISABLE_ILI9481 1 // CTE32HR
