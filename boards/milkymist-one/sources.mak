BOARD_SRC=$(wildcard $(BOARD_DIR)/*.v) 

MINER_SRC=$(wildcard $(CORES_DIR)/fpgaminer/rtl/*.v)

CORES_SRC=$(MINER_SRC)
