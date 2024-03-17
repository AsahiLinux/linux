typedef struct ane_nn ane_nn;
void *pyane_init(char *path, int dev_id);
int pyane_free(ane_nn *nn);
int pyane_exec(ane_nn *nn);
int pyane_send(ane_nn *nn, void **head_pyane_tensor);
int pyane_read(ane_nn *nn, void **head_pyane_tensor);
