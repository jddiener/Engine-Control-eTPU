This is release 3.2 of the eTPU utilities files.
The user should refer to Application Note AN2864 "General C Functions for
the eTPU" which is available from NXP.
____________________________________________________________________________

Files included in this package:
===============================
  util\readme.txt    Rev : 3.2 - This file.
  util\etpu_util.h   Rev : 3.2 - This file contains function prototypes and
                                 useful defines.
  util\etpu_util.c   Rev : 3.2 - This file contains the utilites functions.
  util\etpu_struct.h Rev : 1.3 - This file contains a structure that defines
                                 the eTPU(2) module.
  util\eTPU_util-doxyDoc.chm   - This file contains the documentation generated
                                 from source code by DoxyGen in the format of
                                 Microsoft Compiled HTML Help (CHM)
  util\eTPU_util-doxyDoc.zip   - This file contains the same documentation 
	                               in the format of zip-packed HTML files.
  util\init_template\etpu_gct.c  Rev : 1.1 - This file contains a template code
                                             of eTPU initialization.
  util\init_template\etpu_gct.h  Rev : 1.1 - This file contains a template of
                                             function prototypes and defines 
                                             for etpu_gct.c.
_____________________________________________________________________________

Readme.txt HISTORY
------------------
  Revision 3.2  2014/03/21  r54529
  fs_etpu_clear_chan_interrupt_flag and fs_etpu_clear_chan_dma_flag bug fix
  - the overflow flag was cleared as well.
  fs_etpu_get/clear_chan_interrupt_oveflow_flag and 
  fs_etpu_get/clear_chan_dma_overflow_flag added.

  Revision 3.1  2013/07/16  r54529
  fs_etpu_coherent_read/write_32 bug fix.

  Revision 3.0  2012/05/17  r54529
  fs_etpu2_init bug fix - engine-relative memory granularity is 512 bytes.

  Revision 2.9  2012/05/12  r54529
  fs_etpu2_init bug fix
  
  Revision 2.8  2012/04/10  r54529
  fs_etpu2_init reworked - engine memory allocated but not initialized, becuase
  the compiler never generated staticly allocated engine variables.   

  Revision 2.7  2012/03/13 r54529
  etpu_util.c and etpu_util.h updated to version 2.7.

  Revision 2.6  2012/03/01 r54529
  etpu_util.c and etpu_util.h updated to version 2.6.
  eTPU initialization code template added

  Revision 2.5  2012/02/10 13:54:50  r54529
  etpu_util.c and etpu_util.h updated to version 2.5.
  Documentation generated from source code by DoxyGen added.

  Revision 2.4  2011/07/11 13:20:20  r54529
  etpu_util.c and etpu_util.h updated to preliminary v 2.3.

  Revision 2.3  2009/12/17 20:51:39  r54529
  etpu_util.c and etpu_util.h updated to v 2.2.

  Revision 2.2  2009/10/30 08:35:15  r47354
  Updates to etpu_util.c and etpu_util.h

  Revision 2.1  2004/12/02 14:48:08  r12110
  -Added all of the files to readme.txt

  Revision 2.0  2004/11/30 16:12:22  r12110
  -Updated functions to remove requirement for etpu_config.h file.
  -Added new memset32 function to clear eTPU code memory.



