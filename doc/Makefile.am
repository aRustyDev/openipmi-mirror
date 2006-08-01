
DIA_FILES = BusSystem.dia       ManagementController.dia  SimpleHotSwap.dia \
	    ComplexHotSwap.dia  MgmtIF.dia                Thresh.dia \
	    Hyst.dia            ServerSystem.dia

EXTRA_DIST = IPMI.ltx IPMI.pdf ipmi.bib ${DIA_FILES}

EPS_FILES = $(patsubst %.dia,%.eps,${DIA_FILES})

IPMI.pdf: IPMI.dvi
	${LATEX} IPMI.ltx
	${BIBTEX} IPMI
	${LATEX} IPMI.ltx
	${LATEX} IPMI.ltx

IPMI.dvi: IPMI.ltx ${EPS_FILES}
	$(LATEX) IPMI.ltx

%.eps : %.dia
	$(DIA) --export=$@ $(DIA_FILTER_NAME)=eps $<
