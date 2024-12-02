# -*- coding: utf-8 -*-
import sys
import uuid
import requests
import hashlib
import time
import jieba.posseg as pseg

YOUDAO_URL = 'https://openapi.youdao.com/api'
APP_KEY = '014948837bef39ba'
APP_SECRET = 'ILf5XPeBOzjpwY3NhdK0XKwZmBs6vks5'

def encrypt(signStr):
    hash_algorithm = hashlib.sha256()
    hash_algorithm.update(signStr.encode('utf-8'))
    return hash_algorithm.hexdigest()

def truncate(q):
    if q is None:
        return None
    size = len(q)
    return q if size <= 20 else q[0:10] + str(size) + q[size - 10:size]

def do_request(data):
    headers = {'Content-Type': 'application/x-www-form-urlencoded'}
    return requests.post(YOUDAO_URL, data=data, headers=headers)

def youdao_translate(q):
    data = {}
    data['from'] = 'zh-CHS'
    data['to'] = 'en'
    data['signType'] = 'v3'
    curtime = str(int(time.time()))
    data['curtime'] = curtime
    salt = str(uuid.uuid1())
    signStr = APP_KEY + truncate(q) + salt + curtime + APP_SECRET
    sign = encrypt(signStr)
    data['appKey'] = APP_KEY
    data['q'] = q
    data['salt'] = salt
    data['sign'] = sign

    response = do_request(data)
    response_dict = response.json()
    return response_dict["translation"][0]

def process_nouns(words):
    noun_phrase = []
    translated_nouns = []
    for word, flag in words:
        if 'n' in flag or 'a' in flag:
            noun_phrase.append(word)
        else:
            if noun_phrase:
                translated_noun = youdao_translate(''.join(noun_phrase))
                translated_nouns.append(translated_noun)
                noun_phrase = []
    if noun_phrase:
        translated_noun = youdao_translate(''.join(noun_phrase))
        translated_nouns.append(translated_noun)
    return translated_nouns

if __name__ == '__main__':
    input_text = input("请输入要检测的目标: ")
    words = pseg.cut(input_text)
    translated_nouns = process_nouns(words)
    print("Translated Nouns:", translated_nouns)

# def process_nouns(words):
#     noun_phrase = []
#     for word, flag in words:
#         if 'n' in flag or 'a' in flag:
#             noun_phrase.append(word)
#         else:
#             if noun_phrase:
#                 print(youdao_translate(''.join(noun_phrase)), end=' ')
#                 noun_phrase = []
#     if noun_phrase:
#         print(youdao_translate(''.join(noun_phrase)), end=' ')

# if __name__ == '__main__':
#     input_text = input("input:")
#     words = pseg.cut(input_text)
#     process_nouns(words)