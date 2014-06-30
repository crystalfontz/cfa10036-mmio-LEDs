.PHONY: clean All

All:
	@echo "----------Building project:[ LED_test - Debug ]----------"
	@cd "LED_test" && $(MAKE) -f  "LED_test.mk"
clean:
	@echo "----------Cleaning project:[ LED_test - Debug ]----------"
	@cd "LED_test" && $(MAKE) -f  "LED_test.mk" clean
