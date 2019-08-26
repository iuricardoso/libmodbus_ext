#include <stdio.h>
#include <modbus/modbus.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>
#include <string.h>

#define unit_test(func)\
{\
    printf("*** Starting unit test '%s'...\n", #func);\
    func();\
    printf("*** End of unit test '%s'.\n", #func);\
}

#define printf(...){\
    fprintf(stdout,__VA_ARGS__);\
    fflush(stdout);\
}

int v1[MODBUS_DEFAULT_LIST_CAPACITY];

void init_v1(void);
void test_list_new(void);
void test_list_free(void);
void test_list_add(void);
void test_list_get(void);
void test_list_remove(void);
void test_list_remove_in_the_middle(void);
void test_list_insert(void);
void test_list_clean(void);
void test_list_reorganize(void);
void test_list_capacity(void);

int main(int argc, char *argv[])
{
    init_v1();

    unit_test(test_list_new);
    unit_test(test_list_free);
    unit_test(test_list_add);
    unit_test(test_list_get);
    unit_test(test_list_remove);
    unit_test(test_list_remove_in_the_middle);
    unit_test(test_list_insert);
    unit_test(test_list_clean);
    unit_test(test_list_reorganize);
    unit_test(test_list_capacity);
}

void init_v1(void)
{
    //srand(time(NULL));
    int i,e;
    for(i=0, e=sizeof(v1) / sizeof(int); i<e; i++)
    {
        v1[i] = i * 100;//rand() % 1000;
    }
}

void test_list_new(void)
{

    modbus_list_t * list = modbus_list_new(sizeof(int), MODBUS_DEFAULT_LIST_CAPACITY);
    assert(list != NULL);

    assert(modbus_list_get_size(list) == 0);
    assert(modbus_list_get_capacity(list) == MODBUS_DEFAULT_LIST_CAPACITY);
    assert(! modbus_list_remove_element(list, 0) );
    assert(modbus_list_get_element_size(list) == sizeof(int));

    modbus_list_free(list);
}

void test_list_free(void)
{
    modbus_list_t * list1 = modbus_list_new(sizeof(int), MODBUS_DEFAULT_LIST_CAPACITY);
    assert(list1 != NULL);
    modbus_list_free(list1);

    modbus_list_t * list2 = modbus_list_new(sizeof(int), MODBUS_DEFAULT_LIST_CAPACITY);
    assert(list2 != NULL);
    modbus_list_free(list2);

    assert(list1 == list2);
}

void test_list_add(void)
{
    modbus_list_t * list = modbus_list_new(sizeof(int), MODBUS_DEFAULT_LIST_CAPACITY);
    assert(list != NULL);

    int i, s=sizeof(v1) / sizeof(int);
    printf("Adding %d elements:\n", s);
    for(i=0; i<s; i++)
    {
        printf("\t[%3d] -> %4d\n", i, v1[i]);
        assert(modbus_list_add_element(list, &v1[i]) != -1);
        assert(modbus_list_get_size(list) == (size_t)(i+1));
        assert(modbus_list_get_capacity(list) == MODBUS_DEFAULT_LIST_CAPACITY);
    }

    modbus_list_free(list);
}

void test_list_get(void)
{
    modbus_list_t * list = modbus_list_new(sizeof(int), MODBUS_DEFAULT_LIST_CAPACITY);
    assert(list != NULL);

    int i,s;
    for(i=0, s=sizeof(v1) / sizeof(int); i<s; i++)
    {
        assert(modbus_list_add_element(list, &v1[i]) != -1);
        assert(modbus_list_get_size(list) == (size_t)(i+1));
        assert(modbus_list_get_capacity(list) == MODBUS_DEFAULT_LIST_CAPACITY);
    }

    printf("Checking elements:\n");
    for(i=0; i<s; i++)
    {
        int e;
        printf("\t[%3d] -> %4d\n", i, v1[i]);
        assert(modbus_list_get_element(list, &e, i));
        assert( e == v1[i] );
        assert(modbus_list_get_size(list) == MODBUS_DEFAULT_LIST_CAPACITY);
        assert(modbus_list_get_capacity(list) == MODBUS_DEFAULT_LIST_CAPACITY);
    }

    modbus_list_free(list);
}

void test_list_remove(void)
{
    modbus_list_t * list = modbus_list_new(sizeof(int), MODBUS_DEFAULT_LIST_CAPACITY);
    assert(list != NULL);

    int i,s;
    for(i=0, s=sizeof(v1) / sizeof(int); i<s; i++)
    {
        assert(modbus_list_add_element(list, &v1[i]) != -1);
        assert(modbus_list_get_size(list) == (size_t)(i+1));
        assert(modbus_list_get_capacity(list) == MODBUS_DEFAULT_LIST_CAPACITY);
    }

    printf("Removing elements:\n");
    for(i=s-1; i>=0; i--)
    {
        int e;
        printf("\t[%3d] -> %3d... ", i, v1[i]);
        fflush(stdout);
        assert( modbus_list_get_element(list, &e, i) );
        assert( v1[i] == e );
        assert( modbus_list_remove_element(list, i) != -1 );
        assert( modbus_list_get_size(list) == (size_t)i );
        printf("OK! Checking the others:\n");
        int i2, s2;
        for(i2=0, s2=i-1; i2<s2; i2++)
        {
            printf("\t\t[%3d] -> %3d\n", i2, v1[i2]);
            int e;
            assert( modbus_list_get_element(list, &e, i2) );
            assert( e == v1[i2] );
        }
    }

    modbus_list_free(list);
}

void test_list_remove_in_the_middle(void)
{
    modbus_list_t * list = modbus_list_new(sizeof(int), MODBUS_DEFAULT_LIST_CAPACITY);
    assert(list != NULL);
    int v2[MODBUS_DEFAULT_LIST_CAPACITY];

    int index;
    for(index=0; index<MODBUS_DEFAULT_LIST_CAPACITY; index++)
    {
        v2[index] = v1[index];
        modbus_list_add_element(list, &v2[index]);
    }

    srand(time(NULL));

    int quantity;
    for(quantity=MODBUS_DEFAULT_LIST_CAPACITY; quantity>0; quantity--)
    {
        index = rand() % quantity;

        printf("(%d) Removing element in position %d -> ", MODBUS_DEFAULT_LIST_CAPACITY-quantity, index);

        int eV = v2[index];
        int eL;
        assert( modbus_list_get_element(list, &eL, index) );
        printf("v2[%d] (%d) == list[%d] (%d) ...", index, eV, index, eL);

        memmove(&v2[index], &v2[index+1], (quantity-index-1) * sizeof(int) );
        assert( modbus_list_remove_element(list, index) != -1 );
        assert( eV == eL );


        printf("Ok!\nChecking remaining elements:\n");

        /* Checking remainig elements */
        {
            int s;
            for(index=0, s=quantity-1; index<s; index++)
            {
                assert( modbus_list_get_element(list, &eL, index) );
                printf("v2[%d] (%d) == list[%d] (%d)", index, v2[index], index, eL);
                assert( v2[index] == eL );
                printf("Ok!\n");
            }
        }
    }

    modbus_list_free(list);
}

void test_list_insert(void)
{

    modbus_list_t * list = modbus_list_new(sizeof(int), MODBUS_DEFAULT_LIST_CAPACITY);
    assert(list != NULL);

    srand(time(NULL));

    int i, choice;
    printf("Inserting elements:\n");
    for(i=0, choice = 0; i<MODBUS_DEFAULT_LIST_CAPACITY; i++, choice=(choice+1)%3 )
    {
        int index;
        char * type;
        switch(choice)
        {
            case 0: // insert at the beginning
            {
                index = 0;
                type = "beginning";
                break;
            }
            case 1: // insert in the middle
            {
                index = rand() % modbus_list_get_size(list);
                type = "middle";
                break;
            }
            case 2: // insert at the end
            {
                index = -1;
                type = "end";
                break;
            }
        }

        int eL, eO = i * 1000;
        printf("\t(%3d) [%3d (%s)] -> %3d, inserting... ", i, index, type, eO);
        assert( modbus_list_insert_element(list, &eO, index) != -1 );
        printf("Ok!\nchecking...");
        assert( modbus_list_get_element(list, &eL, index) != -1 );
        assert( eO == eL );
        printf("Ok!\n");
    }

    printf("Removing and Re-insert elements:\n");
    for(i=0; i<MODBUS_DEFAULT_LIST_CAPACITY; i++)
    {
        int before, after;
        printf("\t(%3d) getting it... ", i);
        assert( modbus_list_get_element(list, &before, i) );
        printf("Ok: %d. Now, removing... ");
        assert( modbus_list_remove_element(list, i) != -1);
        printf("Ok! Re-inserting... ");
        assert( modbus_list_insert_element(list, &before, i) != -1 );
        printf("Ok! getting it again...");
        assert( modbus_list_get_element(list, &after, i) != -1 );
        assert( before == after );
        printf("Ok!\n");
    }

    modbus_list_free(list);
}


void test_list_clean(void)
{
    modbus_list_t * list = modbus_list_new(sizeof(int), MODBUS_DEFAULT_LIST_CAPACITY);
    assert(list != NULL);

    int i;
    for(i=0; i<MODBUS_DEFAULT_LIST_CAPACITY; i++)
    {
        assert( modbus_list_add_element(list, &v1[i]) != -1 );
    }

    modbus_list_clean(list);

    assert( modbus_list_get_size(list) == 0 );
    assert( modbus_list_get_capacity(list) == MODBUS_DEFAULT_LIST_CAPACITY );

    modbus_list_free(list);
}

void test_list_reorganize(void)
{
    modbus_list_t * list = modbus_list_new(sizeof(int), MODBUS_DEFAULT_LIST_CAPACITY);
    assert(list != NULL);

    int i;
    printf("Adding elements in the list:\n");
    for(i=0; i<MODBUS_DEFAULT_LIST_CAPACITY; i++)
    {
        int e = v1[i];
        printf("[%d] -> %d\n", i, e);
        assert( modbus_list_add_element(list, &e) != -1 );
    }

    printf("Reorganizing elements... ");
    assert( modbus_list_reorganize(list) );
    printf("Ok!\nChecking elements:\n");
    for(i=0; i<MODBUS_DEFAULT_LIST_CAPACITY; i++)
    {
        int e;
        assert( modbus_list_get_element(list, &e, i) != -1 );
        printf("v1[%d] (%d) == list[%d] (%d)\n", i, v1[i], i, e);
    }

    const int remove = 3;
    printf("Removing and adding %d elements:\n", remove);
    for(i=0; i<remove; i++)
    {
        printf("Removing from front... ");
        assert( modbus_list_remove_element(list, 0) != -1);

        int e = v1[i];
        printf("Ok!\nAdding in the tale: %d...", e);
        assert( modbus_list_add_element(list, &v1[i]) != -1);
        printf("Ok!\n");
    }

    printf("Reorganizing elements... ");
    assert( modbus_list_reorganize(list) );
    printf("Ok!\nChecking elements:\n");
    for(i=remove; i<MODBUS_DEFAULT_LIST_CAPACITY; i++)
    {
        int e;
        assert( modbus_list_get_element(list, &e, i-remove) != -1 );
        printf("v1[%d] (%d) == list[%d] (%d)\n", i, v1[i], i-remove, e);
    }

    for(i=0; i<remove; i++)
    {
        int e;
        assert( modbus_list_get_element(list, &e, MODBUS_DEFAULT_LIST_CAPACITY-remove+i) != -1 );
        printf("v1[%d] (%d) == list[%d] (%d)\n", i, v1[i], MODBUS_DEFAULT_LIST_CAPACITY-remove+i, e);
    }


    modbus_list_free(list);
}

void test_list_capacity(void)
{
    const int capacity = 10;
    modbus_list_t * list = modbus_list_new(sizeof(int), capacity);
    assert(list != NULL);

    assert( modbus_list_get_capacity(list) == (size_t)(capacity) );

    int i,s;
    printf("Inserting elements from array without expanding capacity:\n");
    for(i=0; i<capacity; i++)
    {
        printf("\t[%3d] -> %3d\n", i, v1[i]);
        assert( modbus_list_add_element(list, &v1[i]) != -1 );
    }

    printf("Inserting elements from array with capacity expansion:\n");
    for(i=capacity; i<MODBUS_DEFAULT_LIST_CAPACITY; i++)
    {
        printf("\t[%3d] -> %3d\n", i, v1[i]);
        assert( modbus_list_add_element(list, &v1[i]) != -1 );
    }

    assert( modbus_list_get_capacity(list) == MODBUS_DEFAULT_LIST_CAPACITY + capacity );
    assert( modbus_list_get_size(list) == MODBUS_DEFAULT_LIST_CAPACITY );

    printf("Inserting others elements without expanding capacity:\n");
    for(i=0; i<capacity; i++)
    {
        printf("\t[%3d] -> %3d\n", MODBUS_DEFAULT_LIST_CAPACITY+i, i);
        assert( modbus_list_add_element(list, &i) != -1 );
    }

    assert( modbus_list_get_capacity(list) == MODBUS_DEFAULT_LIST_CAPACITY + capacity );
    assert( modbus_list_get_size(list) == MODBUS_DEFAULT_LIST_CAPACITY + capacity );

    printf("Checking elements added from array:\n");
    for(i=0; i<MODBUS_DEFAULT_LIST_CAPACITY; i++)
    {
        printf("\t[%3d] -> %3d\n", i, v1[i]);
        int e;
        assert( modbus_list_get_element(list, &e, i) );
        assert( e == v1[i] );
    }

    printf("Checking others elements:\n");
    for(s=MODBUS_DEFAULT_LIST_CAPACITY + capacity; i<s; i++)
    {
        printf("\t[%3d] -> ", i);
        int e;
        assert( modbus_list_get_element(list, &e, i) );
        int o = i - MODBUS_DEFAULT_LIST_CAPACITY;
        printf("%d == %d\n", e, o);
        assert( e == o );
    }

    printf("Reducing capacity to %d\n", capacity);
    fflush(stdout);
    modbus_list_set_capacity(list, capacity);

    printf("Checking elements:\n");
    for(i=0; i<capacity; i++)
    {
        printf("\t[%3d] -> %3d\n", i, v1[i]);
        int e;
        assert( modbus_list_get_element(list, &e, i) );
        assert( e == v1[i] );
    }

    modbus_list_free(list);
}
