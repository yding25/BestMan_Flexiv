#!/bin/bash

# 要替换的字符串
old_string="basic_env_py38.yaml"
new_string="basic_env_py38.yaml"

# 替换文件中内容
grep -rl "$old_string" . | while read -r file; do
    sed -i "s/$old_string/$new_string/g" "$file"
done

# 输出结果
echo "Replaced all occurrences of '$old_string' with '$new_string' in the following files:"
grep -rl "$new_string" .