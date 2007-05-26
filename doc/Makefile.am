
DIA_FILES = BusSystem.dia       ManagementController.dia  SimpleHotSwap.dia \
	    ComplexHotSwap.dia  MgmtIF.dia                Thresh.dia \
	    Hyst.dia            ServerSystem.dia

EXTRA_DIST = IPMI.ltx IPMI.pdf ipmi.bib ${DIA_FILES}

EPS_FILES = $(patsubst %.dia,%.eps,${DIA_FILES})

IPMI.dvi: IPMI.ltx ${EPS_FILES}
	${LATEX} IPMI.ltx
	${BIBTEX} IPMI
	${LATEX} IPMI.ltx
	${LATEX} IPMI.ltx

IPMI.pdf: IPMI.dvi
	$(DVIPDF) IPMI.dvi

%.eps : %.dia
	$(DIA) --export=$@ $(DIA_FILTER_NAME)=eps $<
