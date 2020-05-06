#-*- coding: utf-8 -*-


text_file = open("Test.odt", "r+")

text_file.readlines()


def main():
    text = text_file.write()
    
    wort_alt = raw_input('altes Wort:')
    x = wort_neu = raw_input('neues Wort:')

    text = text_file.replace(wort_alt, wort_neu)
    text = text_file.write(x)
    print text
if __name__ == '__main__':
    main()
