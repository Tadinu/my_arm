INSTALLDIR=$(HOME)/bin
CXXFLAGS=-O3

clean:
	rm -f *.o *~ mm2 moving_average 

install:
	@make mm2
	@mv mm2 $(INSTALLDIR)
	@make ma
	@mv moving_average $(INSTALLDIR)
	@cp -f average.awk $(INSTALLDIR)
	@cp -f xcs_pop.sh prepare_pop.sh $(INSTALLDIR)
	@cp -f xcs_rwd.sh prepare_rwd.sh $(INSTALLDIR)
	@cp -f xcs_steps.sh prepare_steps.sh $(INSTALLDIR)
	@cp -f xcs_se.sh prepare_se.sh $(INSTALLDIR)

mm2:
	@g++ $(CXXFLAGS) -static -o mm2 mm2.cpp -lm
	@strip mm2

ma:
	@g++ $(CXXFLAGS) -static -o moving_average moving_average.cpp -lm
	@strip moving_average
