libmodbus_ext
=======================

libmodbus_ext é uma versão extendida da biblioteca libmodbus (https://libmodbus.org/) que permite suporte a interrupções.
Para mais informações sobre a biblioteca original, leia o arquivo `README_ORIGINAL.md`!


INSTALAÇÃO DA BIBLIOTECA NO LINUX
---------------------------------
1. Remova a biblioteca libmodbus original, caso esteja instalada, digitando no console `sudo apt-get remove libmobus*`;
2. Faça o download de todo o conteúdo deste repositório e descompacte-o numa pasta;
3. No terminal, dentro da pasta criada contendo os arquivos deste repositório, digite `./autogen.sh` para gerar o script `configure`;
4. Ainda no terminal, digite `sudo ./configure && make install` para instalar a biblioteca.

COMPILAÇÃO DOS PROGRAMAS DE TESTES DA NOVA BIBLIOTECA NO LINUX
--------------------------------------------------------------
1. Instale o Code::Blocks (`sudo apt-get install codeblocks*`), se já não estiver instalado;
2. No Code::Blocks, abra o projeto `test_libmodbus.cbp` existente na pasta `./test_libmodbus_ext/`
3. Faça um build no projeto, nos 'Build Targets' Master e Slave
4. Verifique a existência dos programas `slave` e `master` existentes na pasta `./test_libmodbus_ext/bin/`

EXPLICAÇÃO SOBRE O FUNCIONAMENTO DOS PROGRAMAS MESTRE E ESCRAVO
---------------------------------------------------------------
Os programas `slave` e `master` funcionam simulando os dispostivos escravo e mestre, respectivamente. Neles, o programa mestre conecta no escravo por meio do endereço de loopback, porém é possível instalar um deles em outro computador desde que este proceda com a instalação da biblioteca e a compilação dos programas.

Ambos os programas apresentam em console uma simulação de leitura de valores de sensores e atuadores. A tela do escravo permite que sensores e atuadores tenham seus valores modificados por meio de teclas apresentadas dentro de colchetes. Observe que as letras maiúsculas fazem os valores crescerem enquanto as letras mínusculas fazem descrescerem. Já a tela do mestre permite apenas modificar os atuadores, porém apresenta os valores do sensores capturados do escravo.

Observe que ao alterar os dados dos sensores e/ou atuadores no escravo, os dados são alterados também no mestre, por meio de mensagens de interrupções enviadas do escravo para o mestre. Ao alterar os dados no mestre, mensagens de escrita de dados são enviadas do mestre ao escravo.
