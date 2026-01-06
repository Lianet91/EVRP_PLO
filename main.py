import sys
import csv
from EVRPTW import modelo

if __name__ == '__main__':

    scenarios = [("Bhigh", 8000),]

    epsilons = [0, 0.4, 0.8, 1.2, 1.6, 2.0, 2.4]

    summary_file = "summary_experiments.csv"

    for scenario_name, batt_cap in scenarios:
        for eps in epsilons:

            print(f"\n\n===== {scenario_name}, epsilon = {eps} min, battery = {batt_cap} mAh =====\n")

            # crear modelo
            Results = modelo(epsilon=eps, battery_capacity=batt_cap)
            Results.build()
            Results.solve()

            # === NUEVO: guardar todo el output del reporte ===
            report_filename = f"REPORT_{scenario_name}_eps_{eps}.txt"
            with open(report_filename, "w", encoding="utf-8") as f:
                old_stdout = sys.stdout
                sys.stdout = f

                Results.report()
                sys.stdout = old_stdout

            print(f"Reporte completo guardado en {report_filename}")
