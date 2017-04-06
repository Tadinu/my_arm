BEGIN{ 
	media =0; 
	N = 0;
	quadrato = 0;
}
{
	media = media + $1;
	quadrato = quadrato + $1*$1;
	N = N + 1;
}
END{ 
	print media/N;
	#print quadrato;
	#printf("%10.2g\t%10.2g\n", (media/N), ( (quadrato/N) - (media/N)*(media/N) );
}

