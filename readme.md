# Расчет пересечения треугольников в пространстве

## Сборка
```bash
git clone --recurse-submodules git@github.com:71frukt/Triangles.git
cd Triangles
cmake -B build -S .
cmake --build build
```


## Запуск

### Google тесты
```bash
❯ build/gtest_triangles
```

### Общий тест
```bash
❯ build/test_general
```

### Формат общего теста

Первым числом подается N - количество треугольников, а затем через пробел соответствующие координаты вершин. В конце выводятся номера треугольников, которые пересекаются с какими-либо другими.

```bash
❯ build/test_general
4
0 0 0  0 0 0  0 0 0
0 0 0  1 0 0  0 1 0 
0 0 1  1 0 1  0 1 1
0 0 0  1 0 0  1 0 0

0 1 3  # <- результат 
```