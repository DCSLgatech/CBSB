# CBSB
 A forked repo from [EECBS](https://github.com/Jiaoyang-Li/EECBS) for implementation of CBS-Budget (CBSB).


The code requires the external library BOOST (https://www.boost.org/). After you installed BOOST and downloaded the source code, go into the directory of the source code and compile it with CMake:
```
mkdir build
cd build
cmake ../
make
```

You can run the code 
```
./cbsb -m ../input/random/random-32-32-20.map -a ../input/random/scen-random/random-32-32-20-random-1.scen -o ../output/test/test.csv -k 10 -t 30 --solver=CBSB --suboptimality=1.2 --bypass=false
```

Alternatively, you are able to run the code:
```
cd ../scripts
./testSolver.sh
```



## License
See license.md for further details.
