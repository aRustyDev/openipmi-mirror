RPM		= rpmbuild
RPMFLAGS	= -ta

SUBDIRS    = include utils lib unix $(GLIB_DIR) cmdlang ui lanserv \
	     sample doc man $(SWIG_DIR)

DIST_SUBDIRS = include utils lib unix glib libedit cmdlang \
	     ui lanserv sample doc man swig

EXTRA_DIST = FAQ TODO README.Force README.MotorolaMXP OpenIPMI.spec.in \
	     OpenIPMI.spec ipmi.init ipmi.sysconf

PKGCONFIGS = OpenIPMIutils.pc OpenIPMI.pc OpenIPMIui.pc OpenIPMIposix.pc \
	     OpenIPMIpthread.pc OpenIPMIcmdlang.pc $(GLIB_PKGCONF)

install-data-local: $(PKGCONFIGS)
	$(mkinstalldirs) $(DESTDIR)$(libdir)/pkgconfig
	$(INSTALL_DATA) $(PKGCONFIGS) $(DESTDIR)$(libdir)/pkgconfig

uninstall-local:
	for i in $(PKGCONFIGS); do \
		rm -f $(DESTDIR)$(libdir)/pkgconfig/$$i; \
	done

rpm: dist
	$(RPM) $(RPMFLAGS) $(distdir).tar.gz < /dev/null

DISTCLEANFILES=OpenIPMI.spec OpenIPMIcmdlang.pc OpenIPMI.pc \
	OpenIPMIglib12.pc OpenIPMIposix.pc OpenIPMIui.pc OpenIPMIglib.pc \
	OpenIPMIpthread.pc OpenIPMIutils.pc
