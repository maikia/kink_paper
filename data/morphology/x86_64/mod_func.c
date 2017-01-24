#include <stdio.h>
#include "hocdec.h"
extern int nrnmpi_myid;
extern int nrn_nobanner_;

extern void _Gfluct_reg(void);
extern void _NMDA_Mg_reg(void);
extern void _ca_reg(void);
extern void _caL3d_reg(void);
extern void _cad_reg(void);
extern void _capump_reg(void);
extern void _gabaa5_reg(void);
extern void _ia_reg(void);
extern void _iahp_reg(void);
extern void _iahp2_reg(void);
extern void _ih_reg(void);
extern void _im_reg(void);
extern void _kca_reg(void);
extern void _km_reg(void);
extern void _kv_reg(void);
extern void _na_reg(void);
extern void _nmda5_reg(void);
extern void _release_reg(void);

void modl_reg(){
  if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
    fprintf(stderr, "Additional mechanisms from files\n");

    fprintf(stderr," Gfluct.mod");
    fprintf(stderr," NMDA_Mg.mod");
    fprintf(stderr," ca.mod");
    fprintf(stderr," caL3d.mod");
    fprintf(stderr," cad.mod");
    fprintf(stderr," capump.mod");
    fprintf(stderr," gabaa5.mod");
    fprintf(stderr," ia.mod");
    fprintf(stderr," iahp.mod");
    fprintf(stderr," iahp2.mod");
    fprintf(stderr," ih.mod");
    fprintf(stderr," im.mod");
    fprintf(stderr," kca.mod");
    fprintf(stderr," km.mod");
    fprintf(stderr," kv.mod");
    fprintf(stderr," na.mod");
    fprintf(stderr," nmda5.mod");
    fprintf(stderr," release.mod");
    fprintf(stderr, "\n");
  }
  _Gfluct_reg();
  _NMDA_Mg_reg();
  _ca_reg();
  _caL3d_reg();
  _cad_reg();
  _capump_reg();
  _gabaa5_reg();
  _ia_reg();
  _iahp_reg();
  _iahp2_reg();
  _ih_reg();
  _im_reg();
  _kca_reg();
  _km_reg();
  _kv_reg();
  _na_reg();
  _nmda5_reg();
  _release_reg();
}
