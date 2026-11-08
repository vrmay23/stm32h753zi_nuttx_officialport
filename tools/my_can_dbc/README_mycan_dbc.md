# MyCAN DBC Editor

This is a graphical tool built with PySide6 to create, manage, and export `.dbc` files compatible with tools like CANalyzer, CANoe, and BusMaster. It supports J1939-compliant fields for ECUs, messages, and signals.

---

## Features

- ECU Tab: Define ECU properties (name, address, manufacturer, function, CAN FD support).
- Messages Tab: Create CAN messages with PGN, priority, size, protocol type, and source/destination.
- Signals Tab: Add signal definitions with bit layout, scaling, endianess, signed type, and value tables.
- Overview Tab: View all structured data in a hierarchical tree.
- Export Tab: Save your project as a standard `.dbc` file.
- Save/Load Project: Use JSON to persist or resume work in progress.

---

## Requirements

```bash
pip install -r requirements.txt
```

Python 3.12 is supported.

---

## Running the Application

Make sure your virtual environment is activated (if using one), then run:

```bash
python main.py
```

---

## Project Structure

```
mycan_dbc/
├── main.py                # Entry point
├── gui.py                 # Main window with tab logic and menu
├── save.py                # Save/load project as JSON
├── tab_ecu.py             # ECU configuration tab
├── tab_msg.py             # Message creation tab
├── tab_signal.py          # Signal definition tab
├── tab_export.py          # DBC export tab
├── tab_overview.py        # Project tree view
├── requirements.txt       # Dependencies (PySide6)
```

---

## Export Format

The exported DBC file will contain:
- BO_ definitions with PGN, priority, and sender
- SG_ entries with proper endianess, signed info, factor/offset, min/max, and unit
- CM_ comments for signal value tables

All compliant with J1939 conventions.

---

## License

MIT License. Use freely and modify to fit your needs.

---

##  Contributions

@vrmay23
