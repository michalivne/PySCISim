/* A common file to be included in all interface files */
%feature("autodoc", "3");
%include "boost_shared_ptr.i"
%include "exception.i"

// setup exceptions handler
%exception {
	try{
		$action
		}
	catch (const char*& e) {
	   SWIG_exception(SWIG_RuntimeError, e);
	 } 
	catch (const std::string& e) {
	   SWIG_exception(SWIG_RuntimeError, e.c_str());
	 } 
	catch (const std::runtime_error& e) {
	   SWIG_exception(SWIG_RuntimeError, e.what());
	 } 
	 catch (const std::invalid_argument& e) {
	   SWIG_exception(SWIG_ValueError, e.what());
	 }
	 catch (const std::out_of_range& e) {
	   SWIG_exception(SWIG_IndexError, e.what());
	 }
	catch(std::exception& e )
	{
		SWIG_exception(SWIG_RuntimeError, e.what());
		return NULL;
	}
	 catch (...) { 
	   SWIG_exception(SWIG_RuntimeError, "Unknown exception");
	 } 
 }
