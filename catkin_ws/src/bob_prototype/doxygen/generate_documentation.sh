# Generates documentation for the project in "~/bob_documentation"
# Will create the directory if it does not already exist
if [ ! -d "generated_documentation" ]; then
	mkdir generated_documentation
fi
doxygen doxygen_config.conf
