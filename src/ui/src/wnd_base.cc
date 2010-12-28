/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui/src/wnd_base.h"
#include "ui/src/ui_class.h"

/* Local headers */
#include "../ablibs.h"
#include "../abimport.h"
#include "../gcc_ntox86/proto.h"

namespace mrrocpp {
namespace ui {
namespace common {

//
//
// KLASA WndBase
//
//


WndBase::WndBase(Interface& _interface) :
	interface(_interface), is_open(false)
{

}

}
} //namespace ui
} //namespace mrrocpp
