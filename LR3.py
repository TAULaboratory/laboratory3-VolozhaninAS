import math

import colorama
import control
from control import matlab
from sympy.physics.quantum.circuitplot import pyplot, np

# Kp = float(input("Введите коэффициент П-регулятора: "))

Kp = float(0.06)

Kpp = float(0.095)
Kpi = float(0.0055)
Kpd = float(0.0085)

# П-регулятор
unit_reg_p = control.tf([Kp], [1])

# ПИД-регулятор
unit_reg_pid = control.tf([Kpd, Kpp, Kpi], [1, 0])

# print("ДАННЫЕ ИСПОЛНИТЕЛЬНОГО УСТРОЙСТВА")
unit_exe_dev = control.tf([20], [5, 1])

# print("ДАННЫЕ ТУРБИНЫ")
unit_tur = control.tf([0.02, 1], [0.5, 1])


# print("ДАННЫЕ ГЕНЕРАТОРА")
unit_gen = control.tf([1], [10, 1])

# print("ДАННЫЕ ОБРАТНОЙ СВЯЗИ")
unit_feed_link = control.tf([1], [1])

print(unit_exe_dev, unit_tur, unit_gen, unit_feed_link)

# Функция САУ без регулятора
res_function = unit_exe_dev * unit_tur * unit_gen
res_function_0 = (unit_exe_dev * unit_tur * unit_gen) / (
        1 + unit_exe_dev * unit_tur * unit_gen * unit_feed_link)  # Функция замкнутой системы
res_function_1 = unit_exe_dev * unit_tur * unit_gen * unit_feed_link  # Функция замкнутой системы в разомкнутом состоянии
print('Функция САУ без регулятора: \n', res_function)

# Функция САУ с П-регулятором
res_function_reg_p = unit_exe_dev * unit_tur * unit_gen * unit_reg_p
back = unit_feed_link
full_link_p = control.feedback(res_function_reg_p, back)
print('Функция САУ с П-регулятором: \n', full_link_p)

# Функция САУ с ПИД-регулятором
res_function_reg_pid = unit_exe_dev * unit_tur * unit_gen * unit_reg_pid
full_link_pid = control.feedback(res_function_reg_pid, back)
print('Функция САУ с ПИД-регулятором: \n', full_link_pid)


# Прямые показатели качества
def Direct_method(function):
    # Вспомогательный массив
    time_line = []
    for i in range(0, 1000000):
        time_line.append(i / 1000)

    # Построение переходной характеристики
    [x, y] = control.step_response(function, time_line)
    pyplot.tight_layout()
    pyplot.plot(x, y, color='#05ffc5', alpha=1, linewidth=3)
    pyplot.grid(color='black', linewidth=0.5, linestyle='-')
    pyplot.title('Переходная характеристика h(t)', color='black', fontsize=10)
    pyplot.xlabel('Time, с', fontsize=8)
    pyplot.ylabel('Amplitude', fontsize=8)
    pyplot.xlim(-1, 150)
    pyplot.ylim(-1, 5)
    pyplot.show()

    # Поиск времени регулирования и установившегося значения
    Xestab = len(time_line) - 1
    Yestab = y[len(time_line) - 1]
    X_estab = Xestab
    Y_estab = 0

    # Итерируемся с конца массива, чтобы найти первую точку, которая выходит из диапазона погрешности (то есть первая точка, где
    # переходная функция входит в диапазон погрешности)
    for i in range(len(time_line) - 1, 0, -1):
        if (y[i] > 1.05 * Yestab or y[i] < 0.95 * Yestab):
            Y_estab = y[i]
            X_estab = i / 1000
            break
    print(colorama.Fore.GREEN + 'Время регулирования (tp): ', X_estab, ', с')
    print('Установившееся значение (hуст): ', Yestab)
    print(colorama.Style.RESET_ALL)

    # Нахождение первого максимума
    firstMax = 0
    timeFirstMax = 0
    for i in range(0, 1000000):
        if (y[i] > firstMax):
            firstMax = y[i]
            timeFirstMax = i
    print(colorama.Fore.GREEN + 'Первый максимум (A1): ', firstMax)
    print('Время первого максимума: ', timeFirstMax / 1000)
    print(colorama.Style.RESET_ALL)

    # Нахождение перерегулирования
    over_regulation = ((firstMax - Yestab) / Yestab) * 100
    print(colorama.Fore.GREEN + 'Перерегулирование: ', over_regulation, '%')
    print(colorama.Style.RESET_ALL)

    # Нахождение первого минимума (итерируемся от точки первого максимума до конца характеристики)
    firstMin = firstMax
    timeFirstMin = 0
    for i in range(timeFirstMax + 1, 1000000):
        if (y[i] < firstMin):
            firstMin = y[i]
            timeFirstMin = i
    # print('Первый минимум: ', firstMin)
    # print('Время первого минимума: ', timeFirstMin / 1000)

    # Нахождение второго максимума (итерируемся от точки первого минимума до конца характеристики)
    secondMax = 0
    timeSecondMax = 0
    for i in range(timeFirstMin + 1, 1000000):
        if (y[i] > secondMax):
            secondMax = y[i]
            timeSecondMax = i
    print(colorama.Fore.GREEN + 'Второй максимум (А2): ', secondMax)
    print('Время второго максимума: ', timeSecondMax / 1000)
    print(colorama.Style.RESET_ALL)

    # Нахождение степени затухания и колебательности
    print(colorama.Fore.GREEN + 'Колебательность: ', X_estab / ((timeSecondMax - timeFirstMax) / 1000))
    print('Степень затухания: ', (firstMax - secondMax) / firstMax)
    print(colorama.Style.RESET_ALL)


# Корневой показатели качества
def Root_method(function):
    control.pzmap(function)
    pyplot.tight_layout()
    pyplot.grid(True)
    pyplot.title('Полюсы функции')
    pyplot.xlabel('Re(w)')
    pyplot.ylabel('Im(w)')
    pyplot.show()

    p = control.poles(function)

    Amin = max(p.real)
    timeReg = abs(3 / Amin)
    print(colorama.Fore.BLUE + 'Время регулирования (tp): ', timeReg, ', с')

    koleb = abs(max(p.imag) / max(p.real))
    print('Степень колебательность: ', koleb)

    over_regulation = math.exp(-math.pi / koleb) * 100
    print('Перерегулирования: ', over_regulation, '%')

    degree_attenuation = 1 - math.exp((-2 * math.pi) / koleb)
    print('Степень затухания: ', degree_attenuation)
    print(colorama.Style.RESET_ALL)


# Частотные показатели качества
def Frequency_method(function, resFunction):
    # function - полная передаточная функция САУ
    # resFunction - передаточная функция внутренней части САУ
    omega = []
    for i in range(0, 1000000):
        omega.append(i / 1000)

    mag, phase, omg = matlab.freqresp(function, omega)

    # Построение АЧХ передаточной функции
    pyplot.tight_layout()
    pyplot.grid(color='black', linewidth=0.5, linestyle='-')
    pyplot.plot(omg, mag, color='#05ff97', alpha=1, linewidth=3)
    pyplot.title('АЧХ', color='black', fontsize=10)
    pyplot.xlabel('Time, с', fontsize=8)
    pyplot.ylabel('Amplitude', fontsize=8)
    pyplot.xlim(-1, 5)
    pyplot.ylim(-10, 11)
    pyplot.show()

    # Нахождение максимума АЧХ для последующего нахождения колебательности
    magMax = 0
    omgMax = 0
    for i in range(0, len(mag) - 1):
        if (mag[i] > magMax):
            magMax = mag[i]
            omgMax = i
    print(colorama.Fore.MAGENTA + 'Колебательность: ', magMax / mag[0])
    print('Резонансная частота: ', omgMax / 1000, 'рад/с')

    # Определение частоты среза
    omgSrez = 0
    for i in range(omgMax, len(mag) - 1):
        if (mag[i] / mag[0] < 1.005 and mag[i] / mag[0] > 1):
            omgSrez = i
            break
    print('Частота среза: ', omgSrez / 1000, 'рад/с')

    # Определение длительности переходного процесса (времени регулирования)
    time_reg = 0
    if (omgSrez != 0):
        time_reg = 2 * math.pi / omgSrez
    print('Диапазон времени регулирования: ', '(', time_reg, ';', 2 * time_reg, ')', 'c')
    print(colorama.Style.RESET_ALL)

    # Определение запаса устойчивости по ЛАЧХ и ЛФЧХ
    control.bode(resFunction, dB=True, deg=True, margins=True, color='#ff630f')
    gm, pm, wcg, wcp = control.margin(resFunction)

    print(colorama.Fore.MAGENTA + "Запас по амплитуде: ", f'{gm:.2f}')
    print("Запас по амплитуде:  ", f'{20 * np.log10(gm):.2f}', ", дБ")
    print("Частота wcp: ", f'{wcp:.2f}', ", рад/с")
    print("Запас по фазе: ", f'{pm:.2f}', ", град.")
    print("Частота wcg: ", f'{wcg:.2f}', ", рад/с")
    print(colorama.Style.RESET_ALL)
    pyplot.tight_layout()
    pyplot.grid(color='black', linewidth=0.5, linestyle='-')
    pyplot.show()


# Интегральные показатели качества
def Integral_method(function):
    # Вспомогательный массив
    time_line = []
    for i in range(0, 1000000):
        time_line.append(i / 1000)

    # Определение массивов по осям переходной характеристики САУ
    [x, y] = control.step_response(function, time_line)

    # Определение установившегося значения
    Yestab = y[len(time_line) - 1]

    # Определение интегралов линейного, модульного и квадратичного
    Jl = round(np.trapz((Yestab - y), x), 5)
    Jm = round(np.trapz(abs((Yestab - y)), x), 5)
    Jk = round(np.trapz((Yestab - y) ** 2, x), 5)

    # Вывод значений в консоль
    print(colorama.Fore.LIGHTYELLOW_EX + 'Линейная интегральная оценка: ', Jl)
    print('Модульная интегральная оценка: ', Jm)
    print('Квадратичная интегральная оценка: ', Jk)
    print(colorama.Style.RESET_ALL)

# САУ без регулятора
# print(colorama.Fore.GREEN + 'САУ без регулятора')
# print(colorama.Style.RESET_ALL)
# Direct_method(res_function_0)
# Root_method(res_function_0)
# Frequency_method(res_function_0, res_function)
# Integral_method(res_function_0)

# САУ с П-регулятором
# print(colorama.Fore.GREEN + 'САУ с П-регулятором')
# print(colorama.Style.RESET_ALL)
# Direct_method(full_link_p)
# Root_method(full_link_p)
# Frequency_method(full_link_p, res_function_reg_p)
# Integral_method(full_link_p)

# САУ с ПИД-регулятором
print(colorama.Fore.GREEN + 'САУ с ПИД-регулятором')
print(colorama.Style.RESET_ALL)
Direct_method(full_link_pid)
Root_method(full_link_pid)
Frequency_method(full_link_pid, res_function_reg_pid)
Integral_method(full_link_pid)
